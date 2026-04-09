# Drone Fleet Controller

## Project Structure

```
drone_controller/
├── include/
│   ├── config.h        ← All tunable parameters — start here
│   ├── packets.h       ← Data structures (mirrors Unity packets)
│   ├── json_parser.h   ← JSON parsing (swap for cJSON on ESP-IDF)
│   ├── transport.h     ← UDP sockets (swap for lwIP on ESP-IDF)
│   ├── controller.h    ← Steering logic — your main work area
│   └── json.hpp        ← nlohmann/json (download separately)
└── src/
    └── main.cpp        ← Fleet loop, one agent per drone
```

## Quick Start (SITL on PC)

### 1. Get nlohmann/json
Download `json.hpp` from https://github.com/nlohmann/json/releases
Place it in `include/json.hpp`

### 2. Build
```bash
mkdir build && cd build
cmake ..
cmake --build .
```

### 3. Run
Start Unity first (hit Play), then:
```bash
./drone_controller
```

## Adding More Drones
Edit `include/config.h`:
```cpp
#define DRONE_COUNT 10   // Change this number only
```
That's it — the fleet loop scales automatically.

## Tuning the Controller
All gains are in `config.h`:
```cpp
#define KP_HEADING   0.015f
#define KD_HEADING   0.005f
#define KP_SPEED     0.5f
```

## Setting Targets
In `main.cpp` after init:
```cpp
agents[0].controller.targetHeading = 90.0f;
agents[0].controller.targetSpeed   = 3.0f;
```

## ESP-IDF Migration Checklist
Only 3 files change — everything else is untouched:

| File          | Change                                      |
|---------------|---------------------------------------------|
| transport.h   | socket() → lwip_socket(), same interface    |
| json_parser.h | nlohmann → cJSON, same function signatures  |
| main.cpp      | main() → app_main(), while → vTaskDelay     |

| File          | No change needed                            |
|---------------|---------------------------------------------|
| config.h      | ✓ Identical                                 |
| packets.h     | ✓ Identical                                 |
| controller.h  | ✓ Identical                                 |

## Implementing INDI
Replace `computeSteer()` and `computeThrottle()` in `controller.h`.
The file has a comment block at the bottom with the steps.
Nothing outside controller.h needs to change.
