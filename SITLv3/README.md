# SAR Drone Swarm Controller — Project Template

## Structure

```
swarm_controller/
│
├── main.cpp              ← Entry point — do not edit
├── SwarmController.h     ← YOUR CODE GOES HERE
├── mission.json          ← Waypoints and search zones
│
├── DroneAgent.h          ← One drone: Kalman + PID + Nav + Socket
├── MissionLoader.h       ← Reads mission.json
├── KalmanNav.h           ← Position filter (GPS + IMU + speed)
├── HeadingPID.h          ← Heading PD controller
├── SpeedP.h              ← Speed P controller
├── WaypointNav.h         ← Waypoint sequencer
└── CMakeLists.txt
```

## Quick Start

```bash
mkdir build && cd build
cmake .. -DSITL_DIR=/path/to/your/SITL
make
./swarm_controller ../mission.json
```

## The Only File You Need to Edit: SwarmController.h

```cpp
void init(std::vector<DroneAgent>& agents, const MissionLoader& mission)
{
    // Called once at startup
    // Assign waypoints, tune PID gains, set up comms
}

void tick(std::vector<DroneAgent>& agents, float dt)
{
    // Called every control loop tick (~50 Hz)
    // Add collision avoidance, AIS avoidance, LoRa coordination here
    for (auto& agent : agents)
        agent.receiveAndTick();   // always call this first
}
```

## Mission JSON Format

```json
{
  "waypoints": [
    { "lat": 54.440, "lon": 18.910 },
    { "lat": 54.430, "lon": 18.910 }
  ],
  "search_zones": [
    {
      "name": "SEARCH_ALPHA",
      "boundary": [
        { "lat": 54.482, "lon": 18.855 },
        ...
      ]
    }
  ]
}
```

If `waypoints` is empty, drones will automatically patrol the boundary of `search_zones[0]`.

## Each DroneAgent Exposes

| Field | Type | Description |
|---|---|---|
| `agent.kalman.lat/lon` | `double` | Best estimated position |
| `agent.kalman.stdLat/stdLon` | `float` | Position uncertainty (metres) |
| `agent.kalman.vx/vz` | `float` | Estimated velocity (m/s) |
| `agent.nav` | `WaypointNav` | Set/read waypoints |
| `agent.headingPID.Kp/Kd` | `float` | Tune heading response |
| `agent.speedP.Kp` | `float` | Tune throttle response |
| `agent.state` | `DroneStatePacket` | Raw GPS, IMU, AIS, LoRa |
| `agent.isLost` | `bool` | No packets for 3s |

## Tuning PID Gains

| Symptom | Fix |
|---|---|
| Overshoots heading, oscillates | Lower `Kp`, raise `Kd` |
| Turns too slowly | Raise `Kp` |
| Speed never reaches target | Raise `speedP.Kp` |
| Throttle oscillates | Lower `speedP.Kp` |

Default values: `Kp=0.008`, `Kd=0.003`, `speedP.Kp=0.4`
