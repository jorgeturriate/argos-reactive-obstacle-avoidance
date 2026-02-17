# Reactive Obstacle Avoidance Controller for ARGoS Foot-Bot

This repository contains a modified version of the default **diffusion controller** from the ARGoS swarm robotics simulator. The implementation introduces an improved reactive obstacle-avoidance strategy inspired by **Braitenberg Vehicles**.

The goal of this work was to replace discrete in-place rotations with smooth, continuous motion while maintaining safe obstacle avoidance.

---

## Project Context

This project was developed as part of a robotics simulation task involving:

- Understanding perception-action loops
- Working with an existing research simulator codebase
- Designing behavior-based control strategies
- Improving navigation efficiency in swarm robots

The implementation modifies the original ARGoS example controller:

https://github.com/ilpincy/argos3-examples

Specifically:
```
controllers/footbot_diffusion/footbot_diffusion.cpp
```

---

## Key Improvements Over the Original Controller

### 1. Continuous Motion Instead of Discrete Rotations

The default controller uses a threshold rule:
- Move forward OR rotate in place.

This often causes:
- Stop-and-go behavior
- Reduced average speed
- Inefficient navigation

The modified controller replaces this with:

👉 Continuous wheel speed modulation  
👉 Smooth steering adjustments  
👉 Higher forward velocity

---

### 2. Vector-Based Sensor Fusion

Obstacle direction is estimated using vector summation of proximity sensors:

- Each sensor contributes a vector based on:
  - Detected intensity
  - Sensor orientation
- The resulting vector encodes:
  - Obstacle direction
  - Obstacle strength

This enables more natural reactive steering.

---

### 3. Forward Braking Component

A braking term proportional to:
```
cos(angle) × obstacle intensity
```

reduces both wheel speeds when obstacles are ahead.

---

### 4. Steering Component

A differential steering term based on:
```
sin(angle) × obstacle intensity
```

generates smooth turns away from obstacles.

---

### 5. Symmetry-Breaking Mechanism

When an obstacle is directly in front, symmetric sensor readings can cancel steering forces, causing the robot to get stuck.

To prevent this:

- A steering bias is injected
- The robot is forced to choose a turning direction

This avoids deadlock situations.

---

## Repository Contents

```footbot_diffusion_original.cpp``` → Original ARGoS example controller
```footbot_diffusion_modified.cpp``` → Improved reactive controller


---

## How to Run the Controller

### 1. Install ARGoS

Follow the official installation instructions:

https://www.argos-sim.info

Example (Ubuntu):

```bash
sudo apt install ./argos3_simulator-3.0.0-x86_64-beta59.deb
```
2. Clone the ARGoS Examples Repository
```
git clone https://github.com/ilpincy/argos3-examples.git
cd argos3-examples
```

3. Replace the Controller File
Copy the modified file into:
```
controllers/footbot_diffusion/
```
Replacing:
```
footbot_diffusion.cpp
```
4. Compile the Examples
```
mkdir build
cd build
cmake ..
make
```
5. Run the Diffusion Experiments
Single robot:
```
argos3 -c experiments/diffusion_1.argos
```
Multiple robots:
```
argos3 -c experiments/diffusion_10.argos
```

## Methodology
The controller follows a behavior-based robotics approach inspired by:

Braitenberg, V. (1984).
Vehicles: Experiments in Synthetic Psychology.

The core idea is to directly map sensor inputs to motor commands without explicit path planning, enabling fast and reactive navigation.

## Skills Demonstrated
- Swarm robotics simulation
- ARGoS framework usage
- Behavior-based control design
- Sensor fusion techniques
- Differential drive kinematics
- C++ robotics programming

## Author
Jorge Victor Turriate Llallire
AI & Computer Vision Engineer
M.Sc. Machine Vision & AI — Université Paris-Saclay

## License
This repository contains modifications of open-source ARGoS example code for educational and research purposes.

