# 🎯 Advanced Missile Guidance Simulation System

```
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
---------------------------------------------------------
```


This project simulates an advanced missile navigation system complete with GPS/INS guidance, Kalman filtering, environmental factors (wind, drag), terrain obstacles, threat detection, and exportable flight path visualization in KML format for Google Earth.

Perfect for testing pathfinding algorithms, simulation of guided weapons, swarm behavior, and 3D geospatial visualization.

**Why did i do this?**
> I was training a Machine Learning Model and was bored while vibing to some classical music and slamming Bud Zeros.... its not based on any system so cant be adapted to anything, its meme only google earth simulator technically

<p align="center">
  <img src="https://github.com/WhiskeyCoder/AMGSS/blob/main/mqdefault.jpg" width="600">
</p>

## 📡 Missile Guidance Paradox (Full Meme Version)

> [The missile knows where it is at all times. It knows this because it knows where it isn't](https://www.youtube.com/watch?v=bZe5J8SVCYQ). By subtracting where it is from where it isn't, or where it isn't from where it is (whichever is greater), it obtains a difference, or deviation. The guidance subsystem uses deviations to generate corrective commands to drive the missile from a position where it is to a position where it isn't, and arriving at a position where it wasn't, it now is. Consequently, the position where it is, is now the position that it wasn't, and it follows that the position that it was, is now the position that it isn't.
In the event that the position that it is in is not the position that it wasn't, the system has acquired a variation, the variation being the difference between where the missile is, and where it wasn't. If variation is considered to be a significant factor, it too may be corrected by the GEA. However, the missile must also know where it was.
The missile guidance computer scenario works as follows. Because a variation has modified some of the information the missile has obtained, it is not sure just where it is. However, it is sure where it isn't, within reason, and it knows where it was. It now subtracts where it should be from where it wasn't, or vice-versa, and by differentiating this from the algebraic sum of where it shouldn't be, and where it was, it is able to obtain the deviation and its variation, which is called error.


---

## 🚀 Features

- ✅ GPS & INS-based Navigation
- ✅ Obstacle Avoidance
- ✅ Jamming Zone Support (WIP)
- ✅ Kalman Filtering for Noise Reduction
- ✅ Swarm Missile Coordination
- ✅ Evasive Maneuvers
- ✅ Altitude Management (Climb → Cruise → Terminal Dive)
- ✅ Realistic Atmospheric Drag, Lift & Wind
- ✅ Shockwave Effect at Impact (KML)
- ✅ Custom Coordinate Launches
- ✅ Waypoint Navigation with Distance Calculation
- ✅ Flight Path Export to Google Earth (`.kml`)
- ✅ Glide mode if out of fuel

---

## 🧠 How It Works (a.k.a. The Missile Knows Where It Is...)

> “The missile knows where it is at all times.  
> It knows this because it knows where it isn't...”  
> — Missile Brain

Let’s break it down:

### Step-by-step Logic:

1. **Initial Setup**:
   - The missile is told:  
     - Where it launched from  
     - Where the target is  
     - And a cruising altitude

2. **INS Simulation**:
   - The missile doesn’t "see" the world.
   - It uses *inertial navigation* — estimating where it *should* be based on movement and direction.

3. **Error Correction**:
   - Over time, INS drifts.
   - The system checks:
     - "Where am I now?"
     - "Where was I supposed to be?"
   - Calculates the **difference** and adjusts trajectory.

4. **The Meme Logic**:
   > The missile knows where it is because it knows where it isn’t.  
   > By subtracting where it is from where it isn’t (or vice versa), it obtains a deviation...

5. **Final Approach**:
   - Once near the target:
     - It initiates a **terminal dive**
     - Uses remaining fuel
     - Or glides dramatically toward impact like a dying anime swordsman
     - Then... **💥 Impact**

---

## 🗺️ KML Output
<p align="center">
  <img src="https://github.com/WhiskeyCoder/AMGSS/blob/main/2025-04-21%2022_18_30-Window.png" width="600">
</p>

The system generates a `.kml` file showing:
- 📍 Launch Point
- 🎯 Target Impact
- 📌 Waypoints
- 🧬 Flight Path (with cruise + descent)
- 🌪️ Terminal Dive
- 💥 Shockwave ring

Open it in **Google Earth** for glorious 3D visualization. Tilt your camera to see altitude trails and impact craters (not really, but soon maybe).

---

## 📦 Requirements

- Python 3.7+
- Modules: `numpy`, `math`, `random`, `datetime`, `time`, `xml.etree.ElementTree`
  
Install `numpy` if needed:

```bash
pip install numpy
```

## 🛠️ Usage
Run the script:
```python new_version.py```

Scenario Menu:
```
[1] Basic test scenario (London to East Anglia)
[2] Advanced test scenario (London to Manchester with waypoints)
[3] Quick test (short distance)
[4] Debug mission (simplified test)
[5] Custom Coordinates (manual lat/lon input)
```
Then Choose:
```
[1] Launch single missile
[2] Launch missile swarm
```

## 🎯 Custom Coordinates
Want to simulate a missile strike anywhere on Earth?
Just select [5], then enter:
- Launch Latitude/Longitude
- Target Latitude/Longitude
- (Optional) Altitude values
You’ll get a full mission log and .kml to view in Earth.

## 📂 Output
- .kml file saved with timestamp
- Includes full flight path, descent, and impact
- Example: missile_path_20250421_230513.kml


> “It subtracts where it is from where it isn't, or where it isn't from where it is — whichever is greater — and uses this to generate a deviation...”
> — Some cursed missile logic programmer, probably


## 🛡️ Legal Stuff (Because GitHub Rules)
This is a simulation.
It is not connected to any real missile platform.
It is for educational, entertainment, and visualization purposes only.
No real-world targeting, warheads, or guidance systems are involved.



