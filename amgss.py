import math
import time
import random
import numpy as np
from datetime import datetime
import xml.etree.ElementTree as ET


def debug_mission():
    """Run a mission with detailed debugging output."""
    print("🧪 Starting debug mission with simplified navigation")

    # Create test scenario
    navigator = create_test_scenario()

    # Display trajectory data
    start_pos = navigator.current_position
    end_pos = navigator.target_position
    distance = start_pos.haversine_distance(end_pos)

    print(f"📍 Start: {start_pos.latitude}, {start_pos.longitude}")
    print(f"🎯 Target: {end_pos.latitude}, {end_pos.longitude}")
    print(f"📏 Total distance: {distance:.2f} meters")
    print(f"📝 Step size: {navigator.base_step_distance} meters")
    print(f"🔢 Theoretical minimum steps: {math.ceil(distance / navigator.base_step_distance)}")

    # Run mission with simplified navigation
    result = navigator.move_towards_target_simple()

    # Detailed results
    print("\n📊 Detailed Mission Summary")
    print("-" * 40)
    for key, value in result.items():
        if isinstance(value, float):
            print(f"  {key}: {value:.2f}")
        else:
            print(f"  {key}: {value}")

    return navigator  # Return for further inspection



class GPSData:
    def __init__(self, latitude, longitude, altitude=10000):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude  # Altitude in meters

    def __sub__(self, other):
        return GPSData(self.latitude - other.latitude, self.longitude - other.longitude, self.altitude - other.altitude)

    def __add__(self, other):
        return GPSData(self.latitude + other.latitude, self.longitude + other.longitude, self.altitude + other.altitude)

    def __eq__(self, other):
        return round(self.latitude, 6) == round(other.latitude, 6) and round(self.longitude, 6) == round(
            other.longitude, 6)

    def __str__(self):
        return f"Latitude: {self.latitude}, Longitude: {self.longitude}, Altitude: {self.altitude}m"

    def clone(self):
        """Create a copy of the current GPS data"""
        return GPSData(self.latitude, self.longitude, self.altitude)

    def haversine_distance(self, other):
        """Calculate the great-circle distance between two points with safeguards for numerical stability."""
        try:
            # Check for extreme values
            if abs(self.latitude) > 90 or abs(self.longitude) > 180 or \
                    abs(other.latitude) > 90 or abs(other.longitude) > 180:
                # Constrain values to valid ranges to prevent overflow
                lat1 = max(-90, min(90, self.latitude))
                lon1 = max(-180, min(180, self.longitude))
                lat2 = max(-90, min(90, other.latitude))
                lon2 = max(-180, min(180, other.longitude))
            else:
                lat1, lon1, lat2, lon2 = self.latitude, self.longitude, other.latitude, other.longitude

            # Earth radius in meters
            R = 6371000

            # Convert to radians
            lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

            # Calculate differences
            dlat = lat2 - lat1
            dlon = lon2 - lon1

            # Use haversine formula with safeguards
            a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2

            # Ensure 'a' is in valid range [0,1] to prevent domain errors
            a = max(0, min(1, a))

            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
            distance = R * c

            return distance
        except (ValueError, OverflowError):
            # Fallback calculation using Euclidean distance as approximation
            # This is less accurate but numerically stable
            x1 = R * math.cos(math.radians(0)) * math.cos(math.radians(self.longitude))
            y1 = R * math.cos(math.radians(0)) * math.sin(math.radians(self.longitude))
            z1 = R * math.sin(math.radians(self.latitude))

            x2 = R * math.cos(math.radians(0)) * math.cos(math.radians(other.longitude))
            y2 = R * math.cos(math.radians(0)) * math.sin(math.radians(other.longitude))
            z2 = R * math.sin(math.radians(other.latitude))

            return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)


class AdvancedNavigationSystem:
    def __init__(self, initial_position, target_position, swarm_size=0, is_swarm_member=False, waypoints=None):
        # Core positioning data
        self.current_position = initial_position
        self.target_position = target_position
        self.previous_position = initial_position.clone()
        self.predicted_position = initial_position.clone()

        # Waypoints for multi-target navigation
        self.waypoints = waypoints or []
        self.current_waypoint_index = 0

        # Historical data for Kalman filtering
        self.position_history = [initial_position.clone()]
        self.velocity_history = [(0, 0, 0)]  # (dlat, dlon, dalt) in m/s
        self.acceleration_history = [(0, 0, 0)]  # (dlat, dlon, dalt) in m/s²

        # Flight parameters
        self.base_step_distance = 5  # meters
        self.scan_radius = 5  # meters for the 8-point scan
        self.max_speed_multiplier = 5
        self.fuel = 5000
        self.velocity = 1
        self.acceleration = 0
        self.max_velocity = 900  # ~Mach 3
        self.total_distance_traveled = 0
        self.start_time = time.time()
        self.path = []

        # Guidance modes
        self.guidance_locked = False
        self.locked_axis = None  # "lat" or "lon"
        self.guidance_mode = "GPS"  # GPS, INS, TERRAIN, IR, RADAR

        # Altitude control
        self.cruise_altitude = 10000
        self.descent_threshold = 10000
        self.descent_rate = 20  # meters per step
        self.stop_threshold = 5
        self.glide_ratio = 15  # glide ratio (distance/altitude loss)

        # Error correction
        self.kalman_gain = 0.5
        self.position_error = 0
        self.cumulative_error = 0
        self.course_correction_threshold = 100  # meters

        # Environmental factors
        self.air_density = 1.0  # multiplier
        self.drag_coefficient = 0.1
        self.lift_coefficient = 0.3
        self.wind_direction = random.uniform(0, 360)  # degrees
        self.wind_strength = random.uniform(0, 5)  # m/s

        # Threat management
        self.threat_detected = False
        self.threat_level = 0  # 0-10
        self.evasive_maneuvers = False
        self.ecm_deployed = False
        self.jamming_zones = [(51.6, -0.1, 5000)]  # lat, lon, radius in meters
        self.obstacles = [
            (51.5, -0.05, 2000, 1000),  # lat, lon, radius, height
            (51.6, 0.2, 3000, 2000)
        ]

        # Swarm management
        self.swarm_size = swarm_size
        self.swarm_missiles = self.create_swarm() if not is_swarm_member else []
        self.swarm_formation = "spread"  # spread, line, circle

        # Special modes
        self.drunk_missile_mode = False
        self.drunk_missile_probability = 0.01  # 1% chance per step
        self.terminal_guidance_active = False
        self.terminal_guidance_distance = 1000  # meters

        # Transcript-based state tracking (the joke)
        self.where_it_is = initial_position.clone()
        self.where_it_isnt = target_position.clone()
        self.where_it_was = initial_position.clone()
        self.where_it_should_be = initial_position.clone()
        self.deviation = 0
        self.variation = 0

        # KML path tracking
        self.kml_path = []

    def move_towards_target_simple(self):
        """
        A simplified navigation function that uses direct movement rather than scan-based pathfinding.
        This function ensures the missile reaches its target reliably, though it may not avoid all obstacles.
        """
        print(f"🚀 Starting simplified navigation with step distance: {self.base_step_distance}m")

        self.kml_path = []
        self.kml_path.append((
            self.current_position.latitude,
            self.current_position.longitude,
            self.current_position.altitude,
            time.time()
        ))
        print(f"✅ Initial position recorded: {self.current_position}")

        initial_distance = self.current_position.haversine_distance(self.target_position)
        print(f"📏 Total distance to target: {initial_distance:.2f}m")

        if self.waypoints:
            print(f"🛣️ Using {len(self.waypoints)} waypoints:")
            for i, waypoint in enumerate(self.waypoints):
                waypoint_dist = self.current_position.haversine_distance(waypoint)
                target_dist = waypoint.haversine_distance(self.target_position)
                print(
                    f"  Waypoint {i + 1}: {waypoint} (Distance: {waypoint_dist:.2f}m, then {target_dist:.2f}m to target)")

        step_count = 0
        print(
            f"✓ Initial position: {self.current_position.latitude}, {self.current_position.longitude}, {self.current_position.altitude}m")

        while ((self.current_position.haversine_distance(self.target_position) > self.stop_threshold or
                self.current_position.altitude > 0) and
               (self.fuel > 0 or self.current_position.altitude > 0)):

            self.adjust_altitude()

            try:
                step_count += 1
                distance_to_target = self.current_position.haversine_distance(self.target_position)

                # Select current target: either waypoint or final target
                if self.waypoints and self.current_waypoint_index < len(self.waypoints):
                    current_target = self.waypoints[self.current_waypoint_index]
                    waypoint_distance = self.current_position.haversine_distance(current_target)
                    print(f"🔄 Tracking waypoint {self.current_waypoint_index + 1}: Distance = {waypoint_distance:.2f}m")

                    if waypoint_distance < self.stop_threshold:
                        print(f"🏁 Waypoint {self.current_waypoint_index + 1} reached!")
                        self.current_waypoint_index += 1

                    if self.current_waypoint_index < len(self.waypoints):
                        current_target = self.waypoints[self.current_waypoint_index]
                    else:
                        current_target = self.target_position
                else:
                    current_target = self.target_position

                self.previous_position = self.current_position.clone()

                target_lat_diff = current_target.latitude - self.current_position.latitude
                target_lon_diff = current_target.longitude - self.current_position.longitude
                step_size_deg = self.base_step_distance / 111111
                magnitude = math.sqrt(target_lat_diff ** 2 + target_lon_diff ** 2)

                if magnitude > 0:
                    norm_lat_diff = target_lat_diff / magnitude
                    norm_lon_diff = target_lon_diff / magnitude
                    lat_step = norm_lat_diff * min(step_size_deg, magnitude)
                    lon_step = norm_lon_diff * min(step_size_deg, magnitude)

                    new_position = GPSData(
                        self.current_position.latitude + lat_step,
                        self.current_position.longitude + lon_step,
                        self.current_position.altitude
                    )

                    distance_moved = self.current_position.haversine_distance(new_position)
                    self.current_position = new_position

                    self.position_history.append(self.current_position.clone())
                    self.kml_path.append((
                        self.current_position.latitude,
                        self.current_position.longitude,
                        self.current_position.altitude,
                        time.time()
                    ))

                    self.total_distance_traveled += distance_moved

                    # Fuel consumption logic
                    fuel_consumption = 0.1
                    if self.current_position.altitude < self.cruise_altitude and self.current_position.altitude > self.previous_position.altitude:
                        fuel_consumption += 0.1
                    self.fuel = max(0, self.fuel - fuel_consumption)

                    self.check_glide_mode()

                    if step_count % 10 == 0 or step_count < 10:
                        current_distance = self.current_position.haversine_distance(current_target)
                        target_type = "waypoint" if current_target != self.target_position else "target"
                        final_distance = self.current_position.haversine_distance(self.target_position)

                        print(f"Step {step_count}: Distance to current {target_type}: {current_distance:.2f}m, "
                              f"To final target: {final_distance:.2f}m, "
                              f"Altitude: {self.current_position.altitude:.2f}m, "
                              f"Fuel: {self.fuel:.2f}")

                # 💥 FINAL DESCENT LOGIC: Directly over target — begin terminal dive until impact
                if self.current_position.haversine_distance(self.target_position) < self.stop_threshold:
                    while self.current_position.altitude > 0:
                        descent_rate = 500
                        self.current_position.altitude -= descent_rate
                        if self.fuel > 0:
                            self.fuel = max(0, self.fuel - 0.2)
                        if self.current_position.altitude < 0:
                            self.current_position.altitude = 0

                        print(f"💥 Terminal dive! Altitude now: {self.current_position.altitude:.2f}m")

                        # ✅ Add to KML path for each descent point
                        self.kml_path.append((
                            self.current_position.latitude,
                            self.current_position.longitude,
                            self.current_position.altitude,
                            time.time()
                        ))

                        time.sleep(0.1)  # Simulate descent time
                    print("🔥💥 TARGET IMPACTED — direct hit confirmed! 💥🔥")
                    break

            except Exception as e:
                print(f"⚠️ Error in navigation: {str(e)}")
                time.sleep(0.01)

        distance_to_target = self.current_position.haversine_distance(self.target_position)
        success = distance_to_target <= self.stop_threshold

        if success:
            print("\n🎯 Target reached!")
            print(f"  Total distance traveled: {self.total_distance_traveled:.2f}m")
            print(f"  Steps used: {step_count}")
            print(f"  Remaining fuel: {self.fuel:.2f}")
            if len(self.kml_path) > 2:
                self.generate_kml_file()
        elif self.fuel <= 0 and self.current_position.altitude <= 0:
            print("\n⚠️ Mission failed: Out of fuel and crashed")
            print(f"  Missed target by: {distance_to_target:.2f}m")
        else:
            print("\n⚠️ Mission aborted or unresolved")
            print(f"  Missed target by: {distance_to_target:.2f}m")

        return {
            "success": success,
            "distance_traveled": self.total_distance_traveled,
            "fuel_remaining": self.fuel,
            "time_elapsed": time.time() - self.start_time,
            "steps": step_count,
            "steps_remaining": '∞',
            "final_altitude": self.current_position.altitude,
            "distance_to_target": distance_to_target
        }

    def create_swarm(self):
        """Create a swarm of missiles with slight position variations."""
        swarm = []
        for i in range(self.swarm_size):
            # Create slight position variations for each swarm member
            offset_lat = random.uniform(-0.0005, 0.0005)
            offset_lon = random.uniform(-0.0005, 0.0005)
            offset_alt = random.uniform(-100, 100)

            initial_pos = GPSData(
                self.current_position.latitude + offset_lat,
                self.current_position.longitude + offset_lon,
                self.current_position.altitude + offset_alt
            )

            # Each swarm member gets the same waypoints and target
            swarm_missile = AdvancedNavigationSystem(
                initial_pos,
                self.target_position,
                swarm_size=0,
                is_swarm_member=True,
                waypoints=self.waypoints.copy() if self.waypoints else None
            )
            swarm.append(swarm_missile)
        return swarm

    def update_kalman_filter(self):
        """
        Apply Kalman filtering to smooth out position estimates with numerical safeguards.
        This combines current GPS readings with predictions based on previous movement.
        """
        # Only apply Kalman if we have enough history
        if len(self.position_history) < 3:
            return

        try:
            # Ensure lat/lon are within valid ranges
            safe_current_lat = max(-90, min(90, self.current_position.latitude))
            safe_current_lon = max(-180, min(180, self.current_position.longitude))

            # Get latest velocity and acceleration (with safeguards)
            vel_lat = self.velocity_history[-1][0] if abs(self.velocity_history[-1][0]) < 1000 else 0
            vel_lon = self.velocity_history[-1][1] if abs(self.velocity_history[-1][1]) < 1000 else 0
            vel_alt = self.velocity_history[-1][2] if abs(self.velocity_history[-1][2]) < 1000 else 0

            acc_lat = self.acceleration_history[-1][0] if abs(self.acceleration_history[-1][0]) < 100 else 0
            acc_lon = self.acceleration_history[-1][1] if abs(self.acceleration_history[-1][1]) < 100 else 0
            acc_alt = self.acceleration_history[-1][2] if abs(self.acceleration_history[-1][2]) < 100 else 0

            # Convert velocity from m/s to degrees/step for lat/lon
            vel_lat_deg = vel_lat / 111111
            vel_lon_deg = vel_lon / (111111 * math.cos(math.radians(min(89, max(-89, safe_current_lat)))))

            # Maximum allowed prediction changes to prevent extreme values
            max_pred_change_deg = 0.01  # ~1km max
            max_pred_change_alt = 100

            # Predict position based on velocity and acceleration (with limits)
            lat_change = vel_lat_deg + 0.5 * acc_lat / 111111
            lon_change = vel_lon_deg + 0.5 * acc_lon / (
                    111111 * math.cos(math.radians(min(89, max(-89, safe_current_lat)))))
            alt_change = vel_alt + 0.5 * acc_alt

            # Cap extreme predictions
            lat_change = max(-max_pred_change_deg, min(max_pred_change_deg, lat_change))
            lon_change = max(-max_pred_change_deg, min(max_pred_change_deg, lon_change))
            alt_change = max(-max_pred_change_alt, min(max_pred_change_alt, alt_change))

            # Calculate predicted position
            predicted_lat = safe_current_lat + lat_change
            predicted_lon = safe_current_lon + lon_change
            predicted_alt = self.current_position.altitude + alt_change

            # Keep altitude in reasonable bounds
            predicted_alt = max(0, min(30000, predicted_alt))

            # Combine prediction with current measurement using Kalman gain
            # The smoother (Kalman gain) blends prediction with current position
            new_lat = safe_current_lat * (1 - self.kalman_gain) + predicted_lat * self.kalman_gain
            new_lon = safe_current_lon * (1 - self.kalman_gain) + predicted_lon * self.kalman_gain
            new_alt = self.current_position.altitude * (1 - self.kalman_gain) + predicted_alt * self.kalman_gain

            # Ensure results stay within valid bounds
            self.current_position.latitude = max(-90, min(90, new_lat))
            self.current_position.longitude = max(-180, min(180, new_lon))
            self.current_position.altitude = max(0, new_alt)

            # Update the predicted position for next iteration
            self.predicted_position = GPSData(predicted_lat, predicted_lon, predicted_alt)

        except (ValueError, OverflowError) as e:
            # Fallback if numerical errors occur
            print(f"⚠️ Kalman filter instability detected: {str(e)}")

            # Don't update in case of errors - keep current position unchanged
            self.predicted_position = self.current_position.clone()

    def calculate_velocity_and_acceleration(self):
        """Calculate current velocity and acceleration based on position history with numerical safeguards."""
        if len(self.position_history) < 2:
            return

        try:
            # Ensure latitude and longitude are within valid ranges
            safe_current_lat = max(-90, min(90, self.current_position.latitude))
            safe_current_lon = max(-180, min(180, self.current_position.longitude))
            safe_prev_lat = max(-90, min(90, self.previous_position.latitude))
            safe_prev_lon = max(-180, min(180, self.previous_position.longitude))

            # Calculate velocity (change in position) with safety checks
            max_allowable_change = 1.0  # Maximum change in degrees per step (unrealistically high but safe cap)

            lat_change = safe_current_lat - safe_prev_lat
            lon_change = safe_current_lon - safe_prev_lon

            # Cap extreme changes to prevent instability
            lat_change = max(-max_allowable_change, min(max_allowable_change, lat_change))
            lon_change = max(-max_allowable_change, min(max_allowable_change, lon_change))

            # Convert to meters
            current_vel_lat = lat_change * 111111
            # Avoid extreme values for cos(latitude) calculation
            cos_lat = math.cos(math.radians(min(89, max(-89, safe_current_lat))))
            current_vel_lon = lon_change * 111111 * cos_lat

            # Altitude change is simpler
            alt_change = self.current_position.altitude - self.previous_position.altitude
            # Cap extreme altitude changes
            max_alt_change = 1000  # meters per step
            current_vel_alt = max(-max_alt_change, min(max_alt_change, alt_change))

            current_velocity = (current_vel_lat, current_vel_lon, current_vel_alt)
            self.velocity_history.append(current_velocity)

            # Calculate acceleration (change in velocity)
            if len(self.velocity_history) >= 2:
                prev_velocity = self.velocity_history[-2]

                # Calculate acceleration components with safeguards
                acc_lat = current_velocity[0] - prev_velocity[0]
                acc_lon = current_velocity[1] - prev_velocity[1]
                acc_alt = current_velocity[2] - prev_velocity[2]

                # Cap extreme accelerations
                max_acc = 100  # m/s²
                current_acc_lat = max(-max_acc, min(max_acc, acc_lat))
                current_acc_lon = max(-max_acc, min(max_acc, acc_lon))
                current_acc_alt = max(-max_acc, min(max_acc, acc_alt))

                current_acceleration = (current_acc_lat, current_acc_lon, current_acc_alt)
                self.acceleration_history.append(current_acceleration)

                # Calculate total velocity magnitude (in m/s) with safeguards
                vel_squared = current_velocity[0] ** 2 + current_velocity[1] ** 2 + current_velocity[2] ** 2
                self.velocity = min(self.max_velocity, math.sqrt(max(0, vel_squared)))

                # Calculate total acceleration magnitude (in m/s²) with safeguards
                acc_squared = current_acceleration[0] ** 2 + current_acceleration[1] ** 2 + current_acceleration[2] ** 2
                self.acceleration = math.sqrt(max(0, acc_squared))

        except (ValueError, OverflowError) as e:
            # Fallback behavior if calculations cause errors
            print(f"⚠️ Numerical instability detected in velocity calculation: {str(e)}")

            # Reset to safe values
            self.velocity = min(self.velocity, self.max_velocity)
            self.acceleration = 0

            # Append safe placeholder values
            self.velocity_history.append((0, 0, 0))
            if len(self.velocity_history) >= 2:
                self.acceleration_history.append((0, 0, 0))

    def calculate_deviation(self):
        """Calculate the deviation as per the transcript logic - where it is vs. where it isn't."""
        # "By subtracting where it is from where it isn't"
        deviation = self.where_it_isnt.haversine_distance(self.where_it_is)
        return deviation

    def calculate_variation(self):
        """Calculate the variation as per the transcript logic - current position vs. previous position."""
        # "The variation being the difference between where the missile is and where the missile wasn't"
        variation = self.where_it_is.haversine_distance(self.where_it_was)
        return variation

    def calculate_error(self):
        """Calculate error as per the transcript logic."""
        # "Error is the difference between deviation and variation"
        # "The algebraic difference between where it shouldn't be and where it is"
        where_it_shouldnt_be = self.where_it_should_be
        error = where_it_shouldnt_be.haversine_distance(self.where_it_is)
        return error

    def generate_scan_points(self):
        """Generate 16 scan points in a circle around the current position with bias toward target."""
        scan_points = []
        # Convert scan radius from meters to degrees (approximately)
        scan_radius_deg = self.scan_radius / 111111  # 1 degree ≈ 111111 meters

        print(f"Generating scan points at {self.scan_radius}m radius ({scan_radius_deg} degrees)")

        # *** FIX: Generate more points (16) for better directional resolution ***
        for angle in range(0, 360, 22):  # 16 points (360/22 ≈ 16)
            rad_angle = math.radians(angle)
            lat_offset = scan_radius_deg * math.sin(rad_angle)
            lon_offset = scan_radius_deg * math.cos(rad_angle)

            # Create new GPS point with current altitude
            scan_point = GPSData(
                self.current_position.latitude + lat_offset,
                self.current_position.longitude + lon_offset,
                self.current_position.altitude
            )
            scan_points.append(scan_point)

        # *** FIX: Add extra points at different distances toward the target direction ***
        # Get current target
        current_target = self.get_current_target()

        # Calculate vector toward target
        target_lat_diff = current_target.latitude - self.current_position.latitude
        target_lon_diff = current_target.longitude - self.current_position.longitude

        # Normalize the vector
        vector_mag = math.sqrt(target_lat_diff ** 2 + target_lon_diff ** 2)
        if vector_mag > 0:
            norm_lat_diff = target_lat_diff / vector_mag
            norm_lon_diff = target_lon_diff / vector_mag

            # Add points at different distances along the target vector
            for dist_fraction in [0.25, 0.5, 0.75, 1.0]:
                target_dist = self.scan_radius * dist_fraction
                target_dist_deg = target_dist / 111111

                # Create point along target vector
                direct_point = GPSData(
                    self.current_position.latitude + (norm_lat_diff * target_dist_deg),
                    self.current_position.longitude + (norm_lon_diff * target_dist_deg),
                    self.current_position.altitude
                )
                scan_points.append(direct_point)

        return scan_points

    def evaluate_scan_point(self, point):
        """
        Evaluate a scan point's suitability for navigation.
        Returns a score (lower is better).
        """
        # Get target - either current waypoint or final destination
        current_target = self.get_current_target()

        # Calculate distance to target
        distance_to_target = point.haversine_distance(current_target)

        # *** FIX: Increase distance weight significantly to prioritize movement toward target ***
        distance_weight = 20.0  # Much higher weight to prioritize distance reduction
        weighted_distance = distance_to_target * distance_weight

        # Check for obstacles
        obstacle_penalty = 0
        for obstacle_lat, obstacle_lon, obstacle_radius, obstacle_height in self.obstacles:
            obstacle_pos = GPSData(obstacle_lat, obstacle_lon)
            obstacle_distance = point.haversine_distance(obstacle_pos)

            # If point is within obstacle radius and below obstacle height
            if obstacle_distance < obstacle_radius and point.altitude < obstacle_height:
                obstacle_penalty = 20000 / (obstacle_distance + 1)  # Stronger obstacle avoidance

        # Check if point is in a jamming zone
        jamming_penalty = 0
        for jammer_lat, jammer_lon, jammer_radius in self.jamming_zones:
            jammer_pos = GPSData(jammer_lat, jammer_lon)
            jammer_distance = point.haversine_distance(jammer_pos)
            if jammer_distance < jammer_radius:
                jamming_penalty = 5000 / (jammer_distance + 1)  # Avoid division by zero

        # *** FIX: Correct bearing calculation ***
        # Calculate bearing from point to target properly
        delta_lon = current_target.longitude - point.longitude
        y = math.sin(math.radians(delta_lon)) * math.cos(math.radians(current_target.latitude))
        x = (math.cos(math.radians(point.latitude)) *
             math.sin(math.radians(current_target.latitude)) -
             math.sin(math.radians(point.latitude)) *
             math.cos(math.radians(current_target.latitude)) *
             math.cos(math.radians(delta_lon)))

        # Direction to target in degrees
        bearing_to_target = (math.degrees(math.atan2(y, x)) + 360) % 360

        # Calculate wind effect - headwind increases penalty, tailwind decreases it
        wind_penalty = 0
        wind_angle_diff = abs((bearing_to_target - self.wind_direction) % 360)
        if wind_angle_diff < 90:  # Tailwind
            wind_penalty = -self.wind_strength * 10
        else:  # Headwind
            wind_penalty = self.wind_strength * 10

        # Add minimal randomness for terrain
        terrain_factor = random.uniform(0.95, 1.05)  # Reduced randomness

        # Handle drunk missile mode (unlikely with probability set to 0)
        if self.drunk_missile_mode and random.random() < self.drunk_missile_probability:
            drunk_factor = random.uniform(0.5, 2.0)
        else:
            drunk_factor = 1.0

        # Calculate optimal altitude factor
        altitude_factor = 1.0
        distance_to_final_target = point.haversine_distance(self.target_position)

        if distance_to_final_target > self.descent_threshold:
            # During cruise phase, reward points that maintain cruise altitude
            altitude_factor = abs(point.altitude - self.cruise_altitude) / 1000 + 1
        else:
            # During descent phase, reward points that follow optimal glide path
            optimal_altitude = distance_to_final_target / self.glide_ratio
            altitude_factor = abs(point.altitude - optimal_altitude) / 500 + 1

        # Terminal guidance - if close to target, prioritize direct approach
        terminal_guidance_factor = 1.0
        if distance_to_final_target < self.terminal_guidance_distance:
            self.terminal_guidance_active = True
            terminal_guidance_factor = 0.5  # Boost priority of direct path

        # *** FIX: Calculate direct movement vector from current position to target ***
        # Calculate vector from current position to target
        target_vector = (current_target.latitude - self.current_position.latitude,
                         current_target.longitude - self.current_position.longitude)

        # Calculate vector from current position to point
        point_vector = (point.latitude - self.current_position.latitude,
                        point.longitude - self.current_position.longitude)

        # Calculate vector magnitudes
        target_vector_mag = math.sqrt(target_vector[0] ** 2 + target_vector[1] ** 2)
        point_vector_mag = math.sqrt(point_vector[0] ** 2 + point_vector[1] ** 2)

        # Calculate cosine similarity (dot product / magnitudes)
        direction_bonus = 0
        if target_vector_mag > 0 and point_vector_mag > 0:
            dot_product = (target_vector[0] * point_vector[0] +
                           target_vector[1] * point_vector[1])
            cosine_sim = dot_product / (target_vector_mag * point_vector_mag)
            direction_bonus = max(0, cosine_sim)  # Between 0 and 1

        # *** FIX: Heavy directional bias to keep missile on course ***
        # Lower value = better score, so 2 - direction_bonus means points aligned with target get close to score of 1
        direction_factor = 2.0 - direction_bonus  # 1.0 to 2.0 (lower is better)

        # Add a progress bonus - greater distance from start position gets a bonus
        progress_bonus = 0

        # *** FIX: Apply strict direct path preference with higher weight ***
        # Comprehensive score formula (lower is better)
        score = ((weighted_distance * terminal_guidance_factor) +
                 obstacle_penalty +
                 jamming_penalty +
                 wind_penalty) * terrain_factor * altitude_factor * drunk_factor * direction_factor

        return score


    def scan_surroundings(self):
        """Scan 8 points around the current position and find the best path."""
        scan_points = self.generate_scan_points()

        # Evaluate each point
        best_point = None
        best_score = float('inf')

        for point in scan_points:
            score = self.evaluate_scan_point(point)
            if score < best_score:
                best_score = score
                best_point = point

        return best_point

    def get_current_target(self):
        """Get the current target (waypoint or final destination)."""
        if self.waypoints and self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        return self.target_position

    def check_waypoint_reached(self):
        """Check if current waypoint is reached, and move to next if so."""
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            return False

        current_waypoint = self.waypoints[self.current_waypoint_index]
        distance_to_waypoint = self.current_position.haversine_distance(current_waypoint)

        if distance_to_waypoint < self.stop_threshold:
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                next_waypoint = self.waypoints[self.current_waypoint_index]
                print(
                    f"🏁 Waypoint {self.current_waypoint_index - 1} reached! Moving to next waypoint at {next_waypoint}")
                return True
            else:
                print(f"🏁 Final waypoint reached! Moving to target at {self.target_position}")
                return True
        return False

    def detect_threat(self):
        """Simulates rare threat detection (5% chance)."""
        return random.random() < 0.05

    def evasive_maneuver_required(self):
        """If a threat is detected, determine if evasion is required (10% chance)."""
        return random.random() < 0.10

    def update_atmosphere(self):
        """Update atmospheric conditions based on altitude."""
        # Air density decreases with altitude
        altitude_km = self.current_position.altitude / 1000
        self.air_density = math.exp(-0.118 * altitude_km)

        # Wind changes with altitude - stronger at higher altitudes
        if random.random() < 0.1:  # 10% chance to change wind
            self.wind_direction = (self.wind_direction + random.uniform(-20, 20)) % 360

            # Wind strength increases with altitude up to jet stream
            if altitude_km < 10:
                self.wind_strength = random.uniform(0, 5 + altitude_km)
            else:
                # Jet stream wind (stronger)
                self.wind_strength = random.uniform(15, 30)

    def adjust_altitude(self):
        """
        Adjust altitude based on distance to target.
        Maintains cruise altitude until descent threshold, then begins descent.
        """
        distance_to_target = self.current_position.haversine_distance(self.target_position)

        if distance_to_target > self.descent_threshold:
            # Cruise phase - rapidly reach and maintain cruise altitude
            if self.current_position.altitude < self.cruise_altitude:
                # Increase to 1000m per step for much faster climbing
                self.current_position.altitude += min(1000, self.cruise_altitude - self.current_position.altitude)
            elif self.current_position.altitude > self.cruise_altitude:
                self.current_position.altitude -= min(100, self.current_position.altitude - self.cruise_altitude)
        else:
            # Descent phase - calculate optimal glide slope
            desired_altitude = (distance_to_target / self.glide_ratio)

            # If current altitude is higher than desired, descend
            if self.current_position.altitude > desired_altitude:
                self.current_position.altitude -= min(
                    self.descent_rate,
                    self.current_position.altitude - desired_altitude
                )
            # If we're below the optimal altitude, try to climb (faster)
            elif self.current_position.altitude < desired_altitude and self.fuel > 0:
                # Increase to 100m per step for faster corrections
                self.current_position.altitude += min(
                    100,  # Much faster ascent rate
                    desired_altitude - self.current_position.altitude
                )
    def check_glide_mode(self):
        """Check if we need to switch to glide mode (out of fuel)."""
        if self.fuel <= 0:
            # Switch to glide mode
            self.guidance_mode = "GLIDE"

            # Calculate max distance we can glide based on current altitude
            max_glide_distance = self.current_position.altitude * self.glide_ratio
            distance_to_target = self.current_position.haversine_distance(self.target_position)

            # If we can't reach the target, calculate how close we'll get
            if max_glide_distance < distance_to_target:
                completion_percentage = (max_glide_distance / distance_to_target) * 100
                print(
                    f"⚠️ Out of fuel! Switching to glide mode. Can reach {completion_percentage:.1f}% of the way to target.")
            else:
                print(
                    f"⚠️ Out of fuel! Switching to glide mode. Should still reach target with {max_glide_distance - distance_to_target:.1f}m of glide range to spare.")

            return True
        return False

    def apply_drag_effects(self):
        """Apply drag effects based on velocity and air density."""
        if self.velocity > 5:
            # Drag force is proportional to velocity squared and air density
            drag_force = self.drag_coefficient * self.air_density * (self.velocity ** 2) / 1000

            # Reduce velocity based on drag
            self.velocity = max(self.velocity - drag_force, 0)

    def toggle_drunk_missile_mode(self):
        """Randomly toggle drunk missile mode with 1% chance."""
        if random.random() < 0.01:  # 1% chance to toggle
            self.drunk_missile_mode = not self.drunk_missile_mode
            if self.drunk_missile_mode:
                print("🥴 Missile entered drunk mode! Flight path may become erratic.")
            else:
                print("🧠 Missile sobered up. Returning to normal flight.")


    def evade(self):
        """Enhanced evasion by analyzing scan points for the safest retreat path."""
        # Get all scan points with a larger scan radius for evasion
        original_scan_radius = self.scan_radius
        self.scan_radius = self.scan_radius * 3  # Triple the scan radius for evasion
        scan_points = self.generate_scan_points()
        self.scan_radius = original_scan_radius  # Restore original radius

        # Prioritize points that move away from the threat direction
        # We'll create a more sophisticated evasion algorithm

        # First, determine the threat direction (randomly for simulation)
        threat_bearing = random.uniform(0, 360)
        print(f"🔍 Threat detected from bearing {threat_bearing:.1f}°")

        # Calculate the opposite direction (where to flee)
        safe_bearing = (threat_bearing + 180) % 360

        # Rank points by their similarity to the safe bearing
        evasion_candidates = []
        for point in scan_points:
            # Skip points that would be inside obstacles
            is_valid = True
            for obstacle_lat, obstacle_lon, obstacle_radius, obstacle_height in self.obstacles:
                obstacle_pos = GPSData(obstacle_lat, obstacle_lon)
                if point.haversine_distance(obstacle_pos) < obstacle_radius and point.altitude < obstacle_height:
                    is_valid = False
                    break

            if not is_valid:
                continue

            # Calculate bearing to this point
            try:
                # Using atan2 for bearing calculation, ensuring inputs are valid
                delta_lon = point.longitude - self.current_position.longitude
                y = math.sin(math.radians(delta_lon)) * math.cos(math.radians(point.latitude))
                x = (math.cos(math.radians(self.current_position.latitude)) *
                     math.sin(math.radians(point.latitude)) -
                     math.sin(math.radians(self.current_position.latitude)) *
                     math.cos(math.radians(point.latitude)) *
                     math.cos(math.radians(delta_lon)))

                point_bearing = (math.degrees(math.atan2(y, x)) + 360) % 360

                # Calculate how close this bearing is to our safe bearing
                # (Closer to safe bearing = better)
                bearing_diff = abs((point_bearing - safe_bearing) % 360)
                if bearing_diff > 180:
                    bearing_diff = 360 - bearing_diff

                # Also prioritize points that gain altitude for safety
                altitude_gain = max(0, point.altitude - self.current_position.altitude)

                # Calculate an evasion score (lower is better)
                evasion_score = bearing_diff - (altitude_gain / 100)

                evasion_candidates.append((point, evasion_score))
            except (ValueError, OverflowError) as e:
                # Skip points that cause math errors
                continue

        # Sort by evasion score (lower is better)
        evasion_candidates.sort(key=lambda x: x[1])

        # Select the best evasion point
        if evasion_candidates:
            evasion_point = evasion_candidates[0][0]

            # Execute evasion - rapid movement and altitude gain if possible
            self.current_position = evasion_point

            # Update history
            self.position_history.append(self.current_position.clone())

            # Add to KML path
            self.kml_path.append((
                self.current_position.latitude,
                self.current_position.longitude,
                self.current_position.altitude,
                time.time()
            ))

            # Try to gain altitude quickly if fuel permits
            if self.fuel > 10:
                altitude_gain = min(300, self.fuel * 5)  # Max 300m gain based on fuel
                self.current_position.altitude += altitude_gain
                self.fuel -= altitude_gain * 0.02  # Rapid climb uses extra fuel

            # Deploy ECM if the threat is serious
            if self.threat_level > 7 and not self.ecm_deployed:
                self.ecm_deployed = True
                print(f"📡 ECM countermeasures deployed! Jamming enemy tracking systems.")

            print(f"🛡️ Evasive maneuver executed! New position: {self.current_position}")

            # After successful evasion, reduce the threat level
            self.threat_level = max(0, self.threat_level - 3)
            if self.threat_level <= 2:
                self.threat_detected = False
                print("✅ Evasion successful! Returning to normal navigation.")
        else:
            # Fallback - just gain altitude and make a random move
            print("⚠️ No safe evasion path found, emergency maneuver initiated")

            # Create a fallback evasion point with significant altitude gain
            fallback_point = self.current_position.clone()
            if self.fuel > 5:
                fallback_point.altitude += 500  # Significant altitude gain
                self.fuel -= 10  # Emergency maneuver uses extra fuel

            # Add small random movement
            fallback_point.latitude += random.uniform(-0.002, 0.002)
            fallback_point.longitude += random.uniform(-0.002, 0.002)

            # Apply the emergency move
            self.current_position = fallback_point

            # Update tracking data
            self.position_history.append(self.current_position.clone())
            self.kml_path.append((
                self.current_position.latitude,
                self.current_position.longitude,
                self.current_position.altitude,
                time.time()
            ))

            print(f"⚠️ Emergency evasion executed! New position: {self.current_position}")

    def generate_kml_file(self):
        """Generate a KML file of the flight path for visualization in Google Earth."""
        import xml.etree.ElementTree as ET
        from xml.dom.minidom import parseString

        if len(self.kml_path) < 2:
            print("⚠️ Not enough path points to generate KML file")
            return

        try:
            from datetime import datetime
            import math
            import time

            # Debug: Unique locations
            unique_coords = set((lat, lon) for lat, lon, _, _ in self.kml_path)
            print(f"ℹ️ KML path contains {len(self.kml_path)} points with {len(unique_coords)} unique locations")

            kml_root = ET.Element("kml", xmlns="http://www.opengis.net/kml/2.2")
            document = ET.SubElement(kml_root, "Document")
            ET.SubElement(document, "name").text = "Missile Flight Path"
            ET.SubElement(document,
                          "description").text = f"""Flight path generated on {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
            Total distance: {self.total_distance_traveled:.2f}m
            Flight time: {time.time() - self.start_time:.2f} seconds
            Final altitude: {self.current_position.altitude:.2f}m
            Remaining fuel: {self.fuel:.2f}"""

            # Styles
            def add_style(style_id, icon_href=None, line_color="ff0000ff", width="4"):
                style = ET.SubElement(document, "Style", id=style_id)
                if icon_href:
                    icon_style = ET.SubElement(style, "IconStyle")
                    ET.SubElement(icon_style, "scale").text = "1.0"
                    icon = ET.SubElement(icon_style, "Icon")
                    ET.SubElement(icon, "href").text = icon_href
                else:
                    line_style = ET.SubElement(style, "LineStyle")
                    ET.SubElement(line_style, "color").text = line_color
                    ET.SubElement(line_style, "width").text = width

            add_style("startStyle", "http://maps.google.com/mapfiles/kml/shapes/placemark_circle_highlight.png")
            add_style("endStyle", "http://maps.google.com/mapfiles/kml/shapes/target.png")
            add_style("waypointStyle", "http://maps.google.com/mapfiles/kml/shapes/target.png")
            add_style("pathStyle", None, "ff0000ff", "4")
            add_style("descentStyle", None, "ffffffff", "3")
            add_style("shockwaveStyle", None, "ff00ffff", "2")

            # Launch & Impact points
            def add_point(name, style, lat, lon, alt):
                p = ET.SubElement(document, "Placemark")
                ET.SubElement(p, "name").text = name
                ET.SubElement(p, "styleUrl").text = f"#{style}"
                point = ET.SubElement(p, "Point")
                ET.SubElement(point, "coordinates").text = f"{lon},{lat},{alt}"

            add_point("Launch Position", "startStyle", *self.kml_path[0][:3])
            add_point("Impact Position", "endStyle", *self.kml_path[-1][:3])

            # Waypoints
            if self.waypoints:
                folder = ET.SubElement(document, "Folder")
                ET.SubElement(folder, "name").text = "Waypoints"
                for i, wp in enumerate(self.waypoints):
                    add_point(f"Waypoint {i + 1}", "waypointStyle", wp.latitude, wp.longitude, wp.altitude)

            # Obstacles
            if self.obstacles:
                folder = ET.SubElement(document, "Folder")
                ET.SubElement(folder, "name").text = "Obstacles"
                for i, (lat, lon, radius, height) in enumerate(self.obstacles):
                    obstacle = ET.SubElement(folder, "Placemark")
                    ET.SubElement(obstacle, "name").text = f"Obstacle {i + 1}"
                    ET.SubElement(obstacle, "description").text = f"Radius: {radius}m, Height: {height}m"
                    style = ET.SubElement(obstacle, "Style")
                    poly = ET.SubElement(style, "PolyStyle")
                    ET.SubElement(poly, "color").text = "4dff0000"
                    ET.SubElement(poly, "outline").text = "1"
                    polygon = ET.SubElement(obstacle, "Polygon")
                    ET.SubElement(polygon, "extrude").text = "1"
                    ET.SubElement(polygon, "altitudeMode").text = "absolute"
                    outer = ET.SubElement(polygon, "outerBoundaryIs")
                    ring = ET.SubElement(outer, "LinearRing")
                    coords = []
                    for angle in range(0, 360, 10):
                        rad = math.radians(angle)
                        radius_deg = radius / (111111 * math.cos(math.radians(lat)))
                        lon_offset = radius_deg * math.cos(rad)
                        lat_offset = radius_deg * math.sin(rad) / math.cos(math.radians(lat))
                        coords.append(f"{lon + lon_offset},{lat + lat_offset},{height}")
                    coords.append(coords[0])
                    ET.SubElement(ring, "coordinates").text = " ".join(coords)

            # Path LineString
            path = ET.SubElement(document, "Placemark")
            ET.SubElement(path, "name").text = "Flight Path"
            ET.SubElement(path, "styleUrl").text = "#pathStyle"
            line = ET.SubElement(path, "LineString")
            ET.SubElement(line, "tessellate").text = "1"
            ET.SubElement(line, "extrude").text = "1"
            ET.SubElement(line, "altitudeMode").text = "absolute"

            stride = max(1, len(self.kml_path) // 1000)
            path_coords = self.kml_path[::stride]
            if self.kml_path[-1] not in path_coords:
                path_coords += self.kml_path[-10:]

            ET.SubElement(line, "coordinates").text = " ".join(
                [f"{lon},{lat},{alt}" for lat, lon, alt, _ in path_coords]
            )

            # Final Descent
            if len(self.kml_path) > 10:
                dive = ET.SubElement(document, "Placemark")
                ET.SubElement(dive, "name").text = "Terminal Dive"
                ET.SubElement(dive, "styleUrl").text = "#descentStyle"
                dive_line = ET.SubElement(dive, "LineString")
                ET.SubElement(dive_line, "tessellate").text = "1"
                ET.SubElement(dive_line, "extrude").text = "1"
                ET.SubElement(dive_line, "altitudeMode").text = "absolute"
                ET.SubElement(dive_line, "coordinates").text = " ".join(
                    [f"{lon},{lat},{alt}" for lat, lon, alt, _ in self.kml_path[-10:]]
                )

            # Shockwave Ring
            lat, lon, _, _ = self.kml_path[-1]
            shock = ET.SubElement(document, "Placemark")
            ET.SubElement(shock, "name").text = "Shockwave Radius"
            ET.SubElement(shock, "styleUrl").text = "#shockwaveStyle"
            poly = ET.SubElement(shock, "Polygon")
            ET.SubElement(poly, "extrude").text = "1"
            ET.SubElement(poly, "altitudeMode").text = "clampToGround"
            outer = ET.SubElement(poly, "outerBoundaryIs")
            ring = ET.SubElement(outer, "LinearRing")
            radius = 0.015  # ≈ 1.5km
            coords = []
            for angle in range(0, 361, 10):
                rad = math.radians(angle)
                lat_offset = radius * math.cos(rad)
                lon_offset = radius * math.sin(rad) / math.cos(math.radians(lat))
                coords.append(f"{lon + lon_offset},{lat + lat_offset},0")
            ET.SubElement(ring, "coordinates").text = " ".join(coords)

            # Save KML
            filename = f"missile_path_{datetime.now().strftime('%Y%m%d_%H%M%S')}.kml"
            xml_string = ET.tostring(kml_root, encoding='utf-8')
            pretty = parseString(xml_string).toprettyxml(indent="  ")
            with open(filename, 'w') as f:
                f.write(pretty)

            print(f"✅ KML file generated successfully: {filename}")

        except Exception as e:
            print(f"❌ Error generating KML file: {str(e)}")

    def launch_swarm(self):
        """Launch all swarm missiles with coordinated attack patterns."""
        if not self.swarm_missiles:
            print("❌ No swarm missiles to launch!")
            return

        print(f"🚀 Launching swarm attack with {len(self.swarm_missiles)} missiles!")

        # Assign different approach vectors to each missile
        approach_angle_separation = 360 / len(self.swarm_missiles)
        swarm_results = []

        for i, missile in enumerate(self.swarm_missiles):
            # Set a unique approach angle for each missile
            approach_angle = i * approach_angle_separation

            # Modify the missile's initial path based on angle
            offset_distance = 0.001  # Approximately 100m
            offset_lat = offset_distance * math.sin(math.radians(approach_angle))
            offset_lon = offset_distance * math.cos(math.radians(approach_angle))

            # Add a waypoint for this missile to encourage it to approach from its angle
            waypoint_distance = self.current_position.haversine_distance(self.target_position) * 0.7
            waypoint_lat = self.target_position.latitude + (offset_lat * waypoint_distance / 100)
            waypoint_lon = self.target_position.longitude + (offset_lon * waypoint_distance / 100)

            # Create the waypoint and add it to this missile's path
            missile.waypoints = [GPSData(waypoint_lat, waypoint_lon, missile.current_position.altitude)]

            # Adjust each missile's parameters slightly to create diversity
            missile.fuel = self.fuel * random.uniform(0.9, 1.1)  # Vary fuel load
            missile.cruise_altitude = self.cruise_altitude * random.uniform(0.95, 1.05)  # Vary altitude
            missile.base_step_distance = self.base_step_distance * random.uniform(0.9, 1.1)  # Vary speed

            # Set a unique name for this missile
            missile_name = f"Missile {i + 1}"

            # Launch this missile
            print(f"🚀 Launching {missile_name} with approach angle {approach_angle:.0f}°")
            result = missile.move_towards_target()

            # Add result to our collection
            result['missile_name'] = missile_name
            result['approach_angle'] = approach_angle
            swarm_results.append(result)

        # Print summary of swarm attack
        print("\n📊 Swarm Attack Summary")
        print("-" * 40)

        successful_hits = sum(1 for r in swarm_results if r.get('success', False))
        print(f"Total missiles: {len(swarm_results)}")
        print(f"Successful hits: {successful_hits}")
        print(f"Success rate: {successful_hits / len(swarm_results) * 100:.1f}%")

        # Return full results for detailed analysis
        return swarm_results


def create_test_scenario():
    """Create a basic test scenario with obstacles and waypoints (London to East Anglia)."""
    # Define initial and target positions
    initial_lat = 51.444233
    initial_lon = -0.189112
    initial_alt = 0  # Start at ground level

    target_lat = 51.753024
    target_lon = 0.493912

    initial_position = GPSData(initial_lat, initial_lon, initial_alt)
    target_position = GPSData(target_lat, target_lon, 0)  # Target at ground level

    # Create waypoints for a more efficient path - about 1/3 and 2/3 of the journey
    waypoints = [
        GPSData(51.55, 0.05, 8000),    # First waypoint at high altitude
        GPSData(51.65, 0.25, 4000),    # Second waypoint starting descent
    ]

    # Create the navigator with enhanced capabilities
    navigator = AdvancedNavigationSystem(
        initial_position,
        target_position,
        swarm_size=3,
        waypoints=waypoints
    )

    # CRITICAL PARAMETERS
    navigator.fuel = 100000
    navigator.base_step_distance = 3000  # 3km steps for faster progress
    navigator.cruise_altitude = 8000     # Higher cruise altitude
    navigator.descent_threshold = 15000  # Start descent when 15km from target
    navigator.stop_threshold = 100       # Consider target reached within 100m

    # Limiting max_steps
    navigator.max_steps = 100            # Should only need ~30 steps with 3km steps

    # Disable random effects for debugging
    navigator.drunk_missile_probability = 0.0

    # Add obstacles
    navigator.obstacles = [
        (51.52, 0.05, 3000, 2000),  # No-fly zone in central London
    ]

    # Add jamming zones
    navigator.jamming_zones = [
        (51.52, 0.05, 5000),  # Jamming in central London
    ]

    return navigator

def create_advanced_test_scenario():
    """Create a more advanced test scenario with multiple waypoints and dynamic terrain."""
    # Define initial and target positions (London to Manchester)
    initial_lat = 51.5074
    initial_lon = -0.1278
    initial_alt = 0  # Start at ground level

    target_lat = 53.4808
    target_lon = -2.2426

    initial_position = GPSData(initial_lat, initial_lon, initial_alt)
    target_position = GPSData(target_lat, target_lon, 0)  # Target at ground level

    # Create waypoints for a complex path
    waypoints = [
        GPSData(51.7519, -0.3370, 8000),  # St Albans
        GPSData(52.2053, -0.7290, 7000),  # Northampton
        GPSData(52.9548, -1.1581, 5000),  # Nottingham
    ]

    # Create the navigator with enhanced capabilities
    navigator = AdvancedNavigationSystem(
        initial_position,
        target_position,
        swarm_size=5,
        waypoints=waypoints
    )

    # MASSIVELY increase fuel and step sizes
    navigator.fuel = 200000  # Much more fuel
    navigator.base_step_distance = 2000  # Much larger steps
    navigator.scan_radius = 2000  # Wider scan
    navigator.cruise_altitude = 8000  # Higher cruise altitude
    navigator.descent_threshold = 20000  # Start descent earlier
    navigator.stop_threshold = 100  # More forgiving arrival threshold

    # This is critical: limit the maximum number of steps
    navigator.max_steps = 2000

    # Reduce drunk missile probability to avoid erratic paths
    navigator.drunk_missile_probability = 0.001  # Lower chance of drunk mode

    # Add more sophisticated obstacles
    navigator.obstacles = [
        # Major cities and airports as no-fly zones
        (51.5074, -0.1278, 5000, 3000),  # London city center
        (51.4700, -0.4543, 8000, 5000),  # Heathrow Airport
        (52.4500, -1.7432, 6000, 4000),  # Birmingham
        (53.3811, -1.4701, 5000, 3000),  # Sheffield
    ]

    # Add more jamming zones
    navigator.jamming_zones = [
        (51.5074, -0.1278, 10000),  # Heavy jamming in London
        (52.4500, -1.7432, 8000),  # Birmingham jamming
    ]

    return navigator


# 4. Fix the main function to allow the simulation to continue longer
if __name__ == "__main__":
    try:
        print("""
               d8888 888b     d888  .d8888b.   .d8888b.   .d8888b.  
              d88888 8888b   d8888 d88P  Y88b d88P  Y88b d88P  Y88b 
             d88P888 88888b.d88888 888    888 Y88b.      Y88b.      
            d88P 888 888Y88888P888 888         "Y888b.    "Y888b.   
           d88P  888 888 Y888P 888 888  88888     "Y88b.     "Y88b. 
          d88P   888 888  Y8P  888 888    888       "888       "888 
         d8888888888 888   "   888 Y88b  d88P Y88b  d88P Y88b  d88P 
        d88P     888 888       888  "Y8888P88  "Y8888P"   "Y8888P"
                                                                                                                            
         ADVANCED MISSILE GUIDANCE SIMULATION SYSTEM
        ---------------------------------------------------------
            ⌁ EARTH-BASED PATHING SIMULATOR • TACTICAL CHAOS
            ⌁ GPS + INS + MEME-LEVEL LOGIC CORE
            ⌁ VISUALIZE. ADJUST. STRIKE. 💥
        ---------------------------------------------------------""")
        print( )
        print("The missile knows where it is at all times.")
        print("It knows this because it knows where it isn't.")
        print("By subtracting where it is from where it isn't (or vice versa), it obtains a deviation.")
        print("The guidance system uses deviations to generate corrective commands to drive the missile")
        print("from a position where it is to a position where it isn't, arriving at a position where it")
        print("wasn't but now is. Consequently, the position where it is, is now the position where it wasn't.")
        print("-" * 50)

        print("\n[1] Basic test scenario (London to East Anglia)")
        print("[2] Advanced test scenario (London to Manchester with waypoints)")
        print("[3] Quick test (short distance)")
        print("\n[4] Debug mission (simplified test)")
        print("\n[5] Customs Cords")
        scenario_choice = input("Select scenario (1/2/3/4/5): ")

        if scenario_choice == "3":
            # Create a short-distance test for quick debugging
            navigator = AdvancedNavigationSystem(
                GPSData(51.5074, -0.1278, 0),  # London
                GPSData(51.5500, -0.1800, 0),  # Nearby location (~5km away)
                swarm_size=0
            )
            navigator.max_steps = 100
            print("Quick test scenario selected: Short distance for debugging")
        elif scenario_choice == "2":
            navigator = create_advanced_test_scenario()
            print("Advanced scenario selected: London to Manchester with waypoints")
            # Set higher max_steps to allow the missile to reach the target
            navigator.max_steps = 3000
        elif scenario_choice == "4":
            navigator = debug_mission()
        elif scenario_choice == "5":
            print("🧭 Custom mission selected (manual coordinates)")
            try:
                start_lat = float(input("Enter launch latitude: ").strip())
                start_lon = float(input("Enter launch longitude: ").strip())
                target_lat = float(input("Enter target latitude: ").strip())
                target_lon = float(input("Enter target longitude: ").strip())
                # Optional: ask altitude?
                start_alt = float(input("Enter launch altitude (default 0): ").strip() or 0)
                target_alt = float(input("Enter target altitude (default 0): ").strip() or 0)
                step_distance = 2000
                print(f"🚀 Launching custom missile mission ({step_distance}m steps)")
                navigator = AdvancedNavigationSystem(
                    initial_position=GPSData(start_lat, start_lon, start_alt),
                    target_position=GPSData(target_lat, target_lon, target_alt),
                    swarm_size=0
                )
                navigator.max_steps = 1000  # Adjust as needed
                navigator.base_step_distance = step_distance
                navigator.move_towards_target_simple()
                navigator.generate_kml_file()

            except ValueError:
                print("⚠️ Invalid coordinates entered. Try again.")

        else:
            navigator = create_test_scenario()
            print("Basic scenario selected: London to East Anglia")
            navigator.max_steps = 2000

        print("\n[1] Launch single missile")
        print("[2] Launch missile swarm")

        choice = input("Enter your choice (1/2): ")

        if choice == "2":
            results = navigator.launch_swarm()

            # Print mission summary
            print("\n📊 Swarm Mission Summary")
            print("-" * 30)
            successful_hits = sum(1 for r in results if r.get('success', False))
            print(f"  Total missiles: {len(results)}")
            print(f"  Successful hits: {successful_hits}")
            print(f"  Success rate: {successful_hits / len(results) * 100:.1f}%")

        else:
            result = navigator.move_towards_target_simple()

            # Print mission summary
            print("\n📊 Mission Summary")
            print("-" * 30)
            for key, value in result.items():
                if isinstance(value, (int, float)):
                    print(f"  {key}: {value:.2f}" if isinstance(value, float) else f"  {key}: {value}")
                else:
                    print(f"  {key}: {value}")

        print("\nCheck the generated KML file (if available) to visualize the flight path in Google Earth.")

    except KeyboardInterrupt:
        print("\n\n⚠️ Mission aborted by user")
    except Exception as e:
        print(f"\n\n❌ Error executing mission: {str(e)}")
        import traceback

        traceback.print_exc()
