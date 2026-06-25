# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

A from-scratch ROS2 implementation of autonomous mobile robot algorithms — sensor fusion, localization, SLAM, and path planning — running on ROS2 Jazzy + Gazebo Harmonic. It also hosts a companion course (C++ and ROS2 modules) and interview prep material.

**Active branch:** `interview-prep-nav2-sensor-fusion-pcl`. The original ROS Noetic version lives on `main`.

## System requirements

| Component | Version |
|-----------|---------|
| Ubuntu | 24.04 LTS |
| ROS2 | Jazzy Jalisco |
| Gazebo | Harmonic (gz-sim 8.x) |
| Build | colcon + ament_cmake |

## Build and run

```bash
# Source ROS2 before any colcon command
source /opt/ros/jazzy/setup.bash

# Build the ROS2 package
colcon build
source install/setup.bash

# Full simulation (Gazebo + RViz2)
ros2 launch robot_description gazebo.launch.py

# Specific world
ros2 launch robot_description gazebo.launch.py world:=slam_district

# Model viewer only (no simulation)
ros2 launch robot_description description.launch.py

# Teleop (second terminal, after sourcing)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Available `world:=` values: `empty` (default), `slam_district`, `office`, `warehouse`, `construction`, `orchard`, `pipeline`, `solar_farm`. A full `.sdf` path is also accepted.

## Course exercises

### C++ exercises (no ROS2 needed)

```bash
cd course/cpp/01_foundations/exercises
cmake -B build && cmake --build build
./build/<exercise_name>
```

Requirements: `g++` with C++17, `cmake` ≥ 3.16. Build artifacts in `exercises/build/` are gitignored.

### ROS2 exercises (isolated colcon builds)

Build each exercise package into `/tmp` to avoid polluting the workspace:

```bash
source /opt/ros/jazzy/setup.bash

PACKAGE_PATH=course/ros2/07_state_estimation_architecture/exercises/state_estimation_architecture_lab
BUILD_ID=ros2_m07

colcon --log-base /tmp/${BUILD_ID}_log build \
  --base-paths "${PACKAGE_PATH}" \
  --build-base /tmp/${BUILD_ID}_build \
  --install-base /tmp/${BUILD_ID}_install

source /tmp/${BUILD_ID}_install/setup.bash
ros2 run state_estimation_architecture_lab measurement_adapter
```

To run an answer package, swap `exercises` → `answers` in `PACKAGE_PATH` and append `_answer` to the package and executable names.

Build all answer packages at once:
```bash
colcon --log-base /tmp/ros2_course_answers_log build \
  --base-paths course/ros2/*/answers/* \
  --build-base /tmp/ros2_course_answers_build \
  --install-base /tmp/ros2_course_answers_install
```

## Architecture

### ROS2 package: `src/robot_description/`

The single ROS2 package. It is pure resource/config — no C++ nodes. It installs URDF xacros, Gazebo worlds, launch files, and RViz configs via `CMakeLists.txt`.

**Data flow (Gazebo ↔ ROS2):**

```
keyboard/nav stack
   │ /cmd_vel (Twist)
   ▼
ros_gz_bridge ──────────────────► Gazebo Harmonic (gz-sim)
                                    DiffDrive plugin → wheel joints, /odom
                                    gpu_lidar (2D) → /front_laser/scan
                                    gpu_lidar (3D) → /top_lidar_3d/points
                                    IMU / GPS / Magnetometer
                                    JointStatePub → /joint_states (gz)
ros_gz_bridge ◄─── all gz topics ──┘
robot_state_publisher ◄─ /joint_states (ros) ─ joint_state_publisher (ROS timer)
rviz2 ◄─ TF tree, sensor topics
```

**Why `/tf` and `/joint_states` are NOT bridged from Gazebo:** DDS delivers gz plugin messages out of order → non-monotonic timestamps → `tf2` buffer clears → RViz blinks. `joint_state_publisher` (a ROS timer) is the sole source of `/joint_states` — always strictly monotonic.

**Startup sequencing:** `TimerAction(5s)` delays `robot_state_publisher`, `joint_state_publisher`, `spawn_robot`, and `rviz2`. The bridge starts immediately so `/clock` is established before any sim-time node starts.

### Robot sensors

| Sensor | Topic | Rate |
|--------|-------|------|
| 2D LiDAR (Hokuyo UST-10) | `/front_laser/scan` | 50 Hz |
| 3D LiDAR (VLP-16 style) | `/top_lidar_3d/points` | 10 Hz |
| IMU | `/imu/data` | 100 Hz |
| GPS | `/gps/fix` | 10 Hz |
| Magnetometer | `/magnetometer` | 50 Hz |

### Course structure

```
course/
├── cpp/        — 8 C++ modules (no ROS2), each with concepts.md, exercises/, answers/, quiz.md
└── ros2/       — 12 ROS2 modules, each with concepts.md, exercises/, answers/, quiz.md
course/review/  — browser-based flashcard review tool (index.html + cards.js)
course/interview_prep/nav2_sensor_fusion_pcl/
                — deep-dive docs and drills for Nav2, sensor fusion, PCL, ROS2 architecture
```

Each C++ module's `answers/` dir uses isolated cmake builds; those build dirs are gitignored. ROS2 module exercises are full colcon packages built in `/tmp`.

## Troubleshooting

**RViz model blinks:** Multiple `/clock` publishers. Check with `ros2 topic info /clock --verbose` and kill stale `parameter_bridge` processes.

**"Moved backwards in time" warnings:** Nodes started before `/clock` arrived. The `TimerAction(5s)` in `gazebo.launch.py` handles this; increase the delay if it persists.

**Robot not visible in Gazebo:** `GZ_SIM_RESOURCE_PATH` must point to the parent of the installed package share so Gazebo can resolve `model://robot_description/meshes/...`. The launch file sets this automatically; verify the package is built and installed.
