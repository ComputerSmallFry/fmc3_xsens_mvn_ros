# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS2 package bridging Xsens motion capture systems (Awinda Suit, Link Suit, Metagloves by Manus) with ROS2. Receives real-time human skeleton data from a Windows MVN Studio server via UDP and publishes it as ROS2 topics and TF transforms. Developed by IIT HRII lab.

## Build & Run

This is an ament_cmake package (C++17) targeting ROS2 Jazzy. It must live inside a colcon workspace's `src/` directory.

```bash
# Build (from colcon workspace root)
colcon build --packages-select xsens_mvn_ros_msgs xsens_mvn_ros
source install/setup.bash

# Run with RViz2 visualization
ros2 launch xsens_mvn_ros xsens_client.launch.py

# Run without RViz, custom port
ros2 launch xsens_mvn_ros xsens_client.launch.py launch_rviz:=false udp_port:=9001

# Launch parameters: model_name (default: skeleton), reference_frame (default: world), udp_port (default: 8001)
```

## CI

GitHub Actions via `industrial_ci` — tests on ROS2 Jazzy. Triggered on push, PR, and manual dispatch. Submodules are checked out automatically.

## Architecture

Two ament packages in this repo:
- `xsens_mvn_ros/` — main package (C++ node + libraries)
- `xsens_mvn_ros_msgs/` — custom message definitions (git submodule, rosidl)
- `xsens_mvn_ros/xsens_mvn_sdk/` — Xsens SDK parser library (git submodule)

### Data Flow

```
MVN Studio (Windows) --UDP--> Socket --> ParserManager --> HumanDataHandler --> ROS2 publishers + TF
```

### Single Node: `xsens_client` (120 Hz)

Publishes:
- `joint_states` (`sensor_msgs/msg/JointState`) — 28 joints x 3 Euler angles (XYZ), converted deg to rad, names prefixed with `{model_name}_`
- `link_states` (`xsens_mvn_ros_msgs/msg/LinkStateArray`) — 25 links with pose, twist, acceleration
- `com` (`geometry_msgs/msg/Point`) — center of mass
- TF broadcasts for each link frame as `{model_name}_{link_name}` relative to `reference_frame`

All publishers are lazy (only publish when subscribers exist, except TF).

### Key Classes

| Class | File | Role |
|-------|------|------|
| `XSensClient` | `src/xsens_client/XSensClient.cpp` | UDP socket management, background data acquisition thread (std::thread), datagram parsing via SDK's `ParserManager` |
| `HumanDataHandler` | `src/xsens_client/HumanDataHandler.cpp` | Thread-safe storage for joint angles, link states (pose/twist/accel), and COM. Namespace: `hrii::ergonomics` |
| `Socket` | `src/xsens_client/Socket.cpp` | UDP/TCP socket wrapper |
| `XSensModelNames` | `include/xsens_mvn_ros/XSensModel.h` | Static definition of 25 link names and 28 joint names matching Xsens MVN segment/joint IDs |

### Skeleton Model

25 links: spine chain (base_link, pelvis, l5, l3, t12, t8, neck, head), bilateral arms (shoulder, upper_arm, forearm, hand), bilateral legs (upper_leg, lower_leg, foot, toe), plus `generic_link`.

28 joints: spinal (l5_s1 through c1_head), bilateral arm (c7_shoulder, shoulder, elbow, wrist), bilateral leg (hip, knee, ankle, ballfoot), plus 6 virtual joints suffixed `_NA`.

### Dependencies

- ROS2: rclcpp, tf2_ros, tf2_eigen, sensor_msgs, geometry_msgs, urdf
- External: Eigen
- Submodules: `xsens_mvn_ros_msgs`, `xsens_mvn_sdk`

## Code Style

C++17. No linter config present. Follows ROS2/ament conventions. Doxygen comments in headers.
