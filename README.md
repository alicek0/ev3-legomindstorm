# EV3 Autonomous Vehicle Convoy System

A 3-vehicle autonomous convoy system built with LEGO Mindstorm EV3, implementing leader-follower architecture with Bluetooth communication, PID-based line following, and automated parking.

## System Architecture

```
[Leader EV3] ──Bluetooth──> [Follower 1 EV3]
     └──────Bluetooth──────> [Follower 2 EV3]
```

- **Leader**: Navigates autonomously, detects road conditions, sends commands to followers
- **Followers**: Maintain safe following distance using ultrasonic PID control, execute commands from leader

## Features

### Leader Vehicle
- **PID Line Following** — Dual color sensor input (left/right), configurable Kp/Ki/Kd constants
- **Obstacle Detection & Lane Changing** — Ultrasonic sensor triggers automatic lane change maneuver
- **Traffic Sign Recognition** via color sensors:
  - 🔴 Red line → enables parking detection mode
  - 🔵 Blue line → temporary stop (3 seconds)
  - 🟡 Yellow crosswalk → slow speed crossing
- **Automated Parking** — Ultrasonic-guided reverse parking with distance threshold
- **Bluetooth Server** — Broadcasts real-time commands to up to 2 follower vehicles

### Follower Vehicles
- **Distance-based Speed Control** — Maintains 20–30cm following distance using PID
- **Synchronized Lane Changes** — Executes lane change on leader command
- **Automated Parking** — Coordinated parking sequence triggered by leader

## Tech Stack

- **Language**: MicroPython (Pybricks)
- **Hardware**: LEGO Mindstorm EV3, Color Sensors (x2), Ultrasonic Sensors (x2)
- **Communication**: Bluetooth (TextMailbox)
- **Control**: PID Controller (line following + distance keeping)

## PID Configuration

| Parameter | Leader | Follower |
|-----------|--------|----------|
| Base Speed | 90 mm/s | 100 mm/s |
| Kp | 2.0 | 2.0 |
| Ki | 0.01 | 0.01 |
| Kd | 0.5 | 0.5 |

## Command Protocol

| Command | Description |
|---------|-------------|
| `RESUME` | Normal line following |
| `STOP` | Immediate stop |
| `SLOW` | Half-speed mode (crosswalk) |
| `CHANGE_LANE_LEFT` | Trigger left lane change |
| `CHANGE_LANE_RIGHT` | Trigger right lane change |
| `PARK` | Begin parking sequence |
| `ALL_PARK_READY` | All vehicles ready to park |

### Synchronized Parking Protocol
1. Leader detects parking space → sends `PARK` to all followers
2. Each follower stops, sends `PARK_READY` to leader
3. Leader waits until all followers confirm ready
4. Leader broadcasts `ALL_PARK_READY` → all vehicles execute parking simultaneously

## Project Structure

```
ev3-legomindstorm/   
├── third_week_follower.py     # Follower vehicle (final)
├── third_week_leader.py       # Leader vehicle (final)
├── lane_change_test.py        # Lane change testing
├── parking_test.py            # Parking maneuver testing
├── color_sensor_test.py       # Color sensor calibration testing
├── bluetooth_server_test.py   # Bluetooth server testing
├── bluetooth_client_test.py   # Bluetooth client testing
└── ...                        # Weekly iteration files
```

## Course

**Embedded Software Design** | Hanyang University ERICA | Fall 2025

Team project (3 members). Each member independently implemented both leader and follower vehicle logic, and took turns operating as the leader vehicle during testing. I was responsible for the full implementation of both `server.py` (leader) and `third_week_follower.py` (follower), including PID tuning, color-based traffic sign detection, Bluetooth communication, and automated parking sequence.

