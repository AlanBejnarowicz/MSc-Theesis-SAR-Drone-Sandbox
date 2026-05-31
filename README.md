# Alan Tomasz Bejnarowicz 4132829
## Master Thesis
### A Distributed Simulation Sandbox for Decentralized ControlDrone Algorithms in Drone Swarms


<div align="center">

# 🚢 Marine Drone Swarm Simulator

**A high-performance simulation sandbox for decentralized control algorithms in autonomous marine drone swarms**

![Unity](https://img.shields.io/badge/Unity-000000?style=for-the-badge&logo=unity&logoColor=white)
![C++](https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)
![C#](https://img.shields.io/badge/C%23-239120?style=for-the-badge&logo=csharp&logoColor=white)
![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20Linux%20%7C%20macOS-lightgrey?style=for-the-badge)

*Master's Thesis Project — University of Southern Denmark (SDU), 2026*

[📹 Demo Video](https://www.youtube.com/watch?v=9BXr4irV290)

</div>

---

## 📖 Overview

This simulator was developed as part of a Master's thesis at the **Maersk Mc-Kinney Moeller Institute, SDU**, targeting maritime Search and Rescue (SAR) applications.

Unlike existing tools (Gazebo/ROS-based Argo and YARA), this platform is **purpose-built for multi-drone swarm simulation** with a direct path to real hardware deployment — no ROS dependency, no OS lock-in.

---

## ✨ Features

### 🌊 Marine Physics
- Multi-point buoyancy system with wave-slope dynamics
- Pierson-Moskowitz spectral wave model
- Keel resistance, propeller slipstream, and full rudder hydrodynamics

### 📡 Sensor Suite
| Sensor | Details |
|--------|---------|
| GPS | 5–10 Hz, Gaussian noise, WGS84 |
| IMU | 200 Hz, 6-DOF, accelerometer + gyroscope |
| Magnetometer | Heading reference with EM noise model |
| AIS | Transponder + receiver, 15 km range |
| LoRa ToF | Inter-drone ranging, ~2 m accuracy at 1 km |

### 📻 LoRa Communication
- Free-space path loss with maritime multipath (η = 2.5)
- Packet collision modelling and half-duplex enforcement
- Time-of-flight ranging for GPS-denied positioning

### 🗺️ AIS Data Replay
- Ingest real-world CSV-formatted AIS recordings
- Catmull-Rom spline interpolation for smooth vessel trajectories
- Built-in GPS spoofing detection via Haversine distance filter

### 🤖 Swarm Controller
- Decentralised grid-based area coverage over LoRa
- Kalman Filter sensor fusion (GPS + IMU + compass + LoRa ToF)
- Physics-inspired inter-drone collision avoidance
- Elliptical safety zone avoidance for AIS vessels

### 🔌 SITL / HITL Support
- Unified UDP/JSON protocol — same code runs in simulation and on hardware
- Per-drone dedicated port pairs, no ROS required

---

## 📊 Performance

Benchmarked on a **mid-range laptop** (AMD Ryzen 5 7530U, integrated graphics):

| Vessels | FPS |
|---------|-----|
| 12 | ~90 |
| 71 | ~60 |
| 103 | ~30 |

> Compiled standalone builds are expected to push these limits further.

---
