# Autonomous Multi-Drone Swarm for Disaster Search & Rescue

An autonomous multi-drone swarm system for disaster search-and-rescue operations, simulated in **PX4 + Gazebo + ROS 2 Humble**. Instead of relying on a single UAV to search a large disaster area, multiple drones coordinate autonomously: a commander drone manages the mission while swarm members divide the search area, detect potential victims, and dynamically reassign tasks if a drone fails or loses communication.

## Overview

- **Commander**: 1x Omnicopter, manages the mission and coordinates swarm members
- **Swarm members**: 5x Holybro X500 drones, divide and cover the search area
- **Coordination**: ROS 2 handles inter-drone communication and task distribution
- **Key feature**: Dynamic task reassignment — if a drone fails or loses comms, its unfinished search region is automatically reassigned to another available drone
- **Goal**: Rapidly cover large disaster areas, detect survivors, handle drone failures, and complete the mission without manual control of every drone

## Architecture

```
                    ┌─────────────────────┐
                    │   Omnicopter         │
                    │   (Commander)         │
                    │   ROS 2 swarm_commander│
                    └──────────┬───────────┘
                               │
                 uXRCE-DDS Agent (port 8888)
                               │
        ┌──────────┬──────────┼──────────┬──────────┐
        │          │          │          │          │
     X500 #1    X500 #2    X500 #3    X500 #4    X500 #5
    (px4_1)     (px4_2)    (px4_3)    (px4_4)    (px4_5)
```

Each PX4 SITL instance runs in Gazebo and exposes its `fmu/in` and `fmu/out` topics over a namespaced uXRCE-DDS session (`/px4_1` … `/px4_5`, `/commander`). The `swarm_commander` ROS 2 node manages arming, offboard mode, mission state, and task reassignment for all six vehicles.

## Stack

| Component | Version/Tool |
|---|---|
| Flight stack | PX4 Autopilot (SITL) |
| Simulator | Gazebo |
| Middleware | ROS 2 Humble |
| Bridge | Micro XRCE-DDS Agent |
| Message types | `px4_msgs` |
| Commander airframe | Omnicopter |
| Swarm airframe | Holybro X500 |

## Repository structure

```
swarm_ws/
├── src/
│   ├── swarm_commander.py   # ROS 2 node: mission state machine, per-drone control, task reassignment
│   └── swarm_sim.py         # Orchestrates PX4 SITL + Gazebo + DDS agent launch for all 6 vehicles
├── .gitignore
└── README.md
```

## Setup

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- PX4 Autopilot (with Gazebo simulation support)
- Micro XRCE-DDS Agent
- `px4_msgs` built in this workspace

### Build

```bash
cd ~/swarm_ws
colcon build
source install/setup.bash
```

### Run the full simulation

```bash
python3 swarm_sim.py
```

This will, in order:
1. Check the PX4/Gazebo environment and `px4_msgs`
2. Start the Micro XRCE-DDS Agent
3. Launch the Omnicopter commander (spawns Gazebo)
4. Launch the 5 X500 slave drones (attach to the same Gazebo instance)
5. Start the ROS 2 `swarm_commander` node
6. Run the mission and print live swarm status

### Ground control (optional but recommended)

Run [QGroundControl](https://qgroundcontrol.com/) alongside the simulation — it auto-connects to all 6 vehicles' MAVLink ports (`14550`–`14555`) and is useful for monitoring arming state, position, and manually intervening if needed.

## Mission state machine

The commander steps every vehicle through: `INITIALIZE → PRE_OFFBOARD → OFFBOARD → ARM → TAKEOFF → SEARCH → (FAILURE/REDISTRIBUTE if triggered) → RETURN`.

## Status

Actively in development — current focus is verifying reliable multi-vehicle arming and stable DDS communication across the commander and all 5 slave drones ahead of Review 2.

## License

*(Add a license, e.g. MIT, if you want this open for reuse.)*
