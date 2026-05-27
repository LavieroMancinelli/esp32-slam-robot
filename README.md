# ESP32 SLAM and Motion Planning Robot

An ESP32-based mobile robot that performs real-time 2D SLAM (Simultaneous Localization and Mapping) to build a floor plan of an indoor environment, with planned support for autonomous path planning and navigation.

## Overview

The robot uses a VL53L4CD time-of-flight distance sensor mounted on a servo motor to perform 180° range scans at each position. Successive scans are matched using the tangent-based scan matching algorithm described by Lu & Milios (1997) to estimate the robot's motion and build a global occupancy map. The map is served and updated live over WiFi as a webpage viewable from any browser on the local network.

## Hardware

- **Microcontroller**: ESP32-C6-DevKitC-1
- **Motor driver**: TB6612FNG dual H-bridge
- **Distance sensor**: VL53L4CD time-of-flight (I2C)
- **Servo**: Standard PWM servo for sensor sweep
- **Motors**: Two DC gear motors (differential drive) + one free-rolling caster wheel

## Features

- 180° range scan using servo-mounted ToF sensor
- Scan-to-scan matching via golden section search over rotation angle with least-squares translation solve
- Live occupancy-grid map served over WiFi at `192.168.4.1` -- connect to the `carlsjr` network and open a browser to view
- Multi-iteration color coding on the map (each scan iteration rendered in a different color)
- Outlier rejection based on normal direction similarity and point distance thresholds

## Algorithm

Scan matching follows the tangent-based method adapted from:

> Lu, F. & Milios, E. (1997). *Robot Pose Estimation in Unknown Environments by Matching 2D Range Scans*. Journal of Intelligent and Robotic Systems, 18, 249–275.

The key steps per iteration:
1. Collect a 180° range scan in polar form
2. Convert to Cartesian coordinates in the robot's local frame
3. For each point in the new scan, find its correspondence in the previous scan by interpolating at angle θ+ω (the scan point's polar angle offset by trial rotation ω)
4. Reject correspondence outliers based on normal direction mismatch and distance thresholds
5. Use golden section search over ω to minimize the total matching distance `E_match(ω)`
6. At each ω evaluation, solve for optimal translation T via least squares from the correspondence equations
7. Apply the found (ω, T) to transform the new scan into global space and update the map

## Planned Features

- Sample-based motion planning (RRT or similar) for autonomous navigation through the mapped environment
- Obstacle avoidance using the live occupancy map
- Full room mapping over many iterations

## Building and Flashing

Requires ESP-IDF v5.x or later.

```powershell
# From esp-idf directory
.\export.ps1

# From project directory
idf.py set-target esp32c6
idf.py build
idf.py -p COM4 flash monitor # replace COM4 with usb port connected to esp32 board
```

## Configuration

Key parameters in `config.h`:

| Define | Default | Description |
|--------|---------|-------------|
| `MAP_SIZE` | 250 | Map grid dimensions (250×250 cells) |
| `MAP_RATIO` | 5 | mm per map cell |
| `SENSOR_FREQ` | 50 | Range scan samples per sweep |
| `SENSOR_PERIOD` | 2000 | Sweep duration in ms |
| `SPIKE_THRESHOLD` | 30 | mm jump to treat as surface discontinuity |
| `MAX_DISTANCE_PER_ITERATION` | 50 | Distance outlier threshold (Hd) |
| `CORRESP_NORMAL_SIMILARITY` | 30 | Normal direction similarity threshold in degrees (α) |
| `MAXIMUM_UNCERTAINTY_INTERVAL` | 0.1 | Golden section search convergence threshold in degrees |
| `MOVE_SPEED` | 25 | Motor PWM speed (0–99) |
| `MOVE_TIME_PER_STEP` | 500 | ms to drive forward between scans |

## Viewing the Map

1. Connect your device to the WiFi network `carlsjr` (password: `password123`)
3. Open a browser and go to `192.168.4.1`

The map updates every second. Each scan iteration is rendered in a different color (red = iteration 1, blue = iteration 2, etc.).

## References

- Lu, F. & Milios, E. (1997). Robot Pose Estimation in Unknown Environments by Matching 2D Range Scans. *Journal of Intelligent and Robotic Systems*, 18, 249–275.

## Credits

- VL53L4CD ULD driver by [LooUQ](https://github.com/LooUQ/st_vl53l4cd)