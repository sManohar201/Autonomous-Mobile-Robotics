# Autonomous-Mobile-Robotics

From-scratch ROS2 implementation of robot autonomy algorithms — sensor fusion, localization, SLAM, and path planning.

> **Branch `ros2-jazzy`** — Active port from ROS Noetic → ROS2 Jazzy + Gazebo Harmonic.
> For the original ROS Noetic version, see the `main` branch.

---

## System Requirements

| Component | Version |
|-----------|---------|
| Ubuntu | 24.04 LTS |
| ROS | Jazzy Jalisco |
| Gazebo | Harmonic (gz-sim 8.x) |
| Build system | colcon + ament_cmake |

## Installation

```bash
# Install ROS2 Jazzy dependencies
sudo apt install ros-jazzy-ros-gz ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-teleop-twist-keyboard

# Clone and build
git clone git@github.com:sManohar201/Autonomous-Mobile-Robotics.git
cd Autonomous-Mobile-Robotics
git checkout ros2-jazzy
colcon build
source install/setup.bash
```

## Usage

### Full Simulation (Gazebo + RViz2)

```bash
ros2 launch robot_description gazebo.launch.py
```

This starts:
1. **Gazebo Harmonic** — physics simulation with DiffDrive, LiDAR, and joint state plugins
2. **ros_gz_bridge** — bridges `/clock`, `/cmd_vel`, `/odom`, and `/front_laser/scan` between Gazebo and ROS2
3. **robot_state_publisher** — publishes the TF tree from URDF + joint states
4. **joint_state_publisher** — publishes joint positions (sole source, monotonic timestamps)
5. **RViz2** — visualization with robot model, laser scan, and path displays

To drive the robot, open a second terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Model Viewer Only (no simulation)

```bash
ros2 launch robot_description description.launch.py
```

Opens RViz2 with `joint_state_publisher_gui` sliders — useful for inspecting the URDF without Gazebo.

## Architecture

```
                                          ┌─────────────────┐
keyboard / nav stack                      │  Gazebo Harmonic │
     │                                    │  (gz-sim)        │
     │ /cmd_vel (ROS2 Twist)              │                  │
     ▼                                    │  DiffDrive plugin│
ros_gz_bridge ──────────────────────────► │  → wheel joints  │
                                          │  → /odom (gz)    │
                    /odom (ROS2 Odometry) │                  │
robot_state_pub  ◄──────────────────────  │                  │
rviz2            ◄─  ros_gz_bridge  ◄───  │  gpu_lidar       │
                    /front_laser/scan     │  → /front_laser/ │
                    /joint_states         │    scan (gz)      │
                    /clock                │                  │
                                          │  JointStatePub   │
                                          │  → /joint_states │
                                          └─────────────────┘
```

### Topic Bridge Map (Gazebo ↔ ROS2)

| Topic | Message Type | Direction |
|-------|-------------|-----------|
| `/clock` | `Clock` | gz → ros |
| `/cmd_vel` | `Twist` | ros → gz |
| `/odom` | `Odometry` | gz → ros |
| `/front_laser/scan` | `LaserScan` | gz → ros |

**Not bridged (intentionally):**

| Topic | Reason |
|-------|--------|
| `/tf` | DiffDrive publishes `odom→base_link` TF with Gazebo timestamps. DDS delivers these out of order → non-monotonic timestamps → tf2 buffer clears → RViz blinks. Will be replaced by `diff_drive_controller` once `ros2_control` is wired up. |
| `/joint_states` | Same DDS ordering issue. `joint_state_publisher` (ROS timer) is the sole source — always strictly monotonic. |

### Startup Sequencing

ROS nodes with `use_sim_time=true` start on wall clock (~1.7 billion seconds). Gazebo sim time starts at ~0. If nodes start before `/clock` is established, they see a massive backward time jump.

**Solution:** `TimerAction(5s)` delays `robot_state_publisher`, `joint_state_publisher`, `spawn_robot`, and `rviz2`. The bridge starts immediately to deliver `/clock` first.

## Package Structure

```
src/robot_description/
├── urdf/
│   ├── automaton.urdf.xacro    # Main robot model (chassis, wheels, mounts)
│   ├── automaton.gazebo         # Gazebo Harmonic plugins (DiffDrive, JointStatePub)
│   └── laser.urdf.xacro        # LiDAR sensor macro (gpu_lidar)
├── meshes/
│   ├── base.stl                # Chassis mesh
│   ├── wheel.stl               # Wheel mesh
│   ├── hokuyo_ust10_lidar.stl  # LiDAR mesh
│   └── d435.dae                # Depth camera mesh
├── worlds/
│   └── automaton_world.sdf     # Gazebo world (loads Sensors, Imu, Physics systems)
├── launch/
│   ├── gazebo.launch.py        # Full simulation launch
│   └── description.launch.py   # Standalone model viewer
├── rviz_config/
│   └── automaton.rviz          # RViz2 display configuration
├── package.xml
└── CMakeLists.txt
```

## Port Progress

### Phase 1: Robot Description & Simulation ✅

- [x] `package.xml` → ament_cmake format 3
- [x] `CMakeLists.txt` → ament_cmake
- [x] Gazebo plugins → Harmonic API (`gz-sim-diff-drive-system`, `gz-sim-joint-state-publisher-system`)
- [x] LiDAR → `gpu_lidar` sensor type (no embedded ROS plugin)
- [x] World SDF with required system plugins (Sensors, Imu, Physics)
- [x] Launch files → ROS2 Python launch API
- [x] RViz config → `rviz_default_plugins` / `rviz_common`
- [x] ros_gz_bridge topic bridging with DDS timestamp fixes
- [x] Teleop keyboard control verified

### Phase 2: Perception (not started)

- [ ] `scan_merger` — combine front/rear laser (message_filters, TF2)
- [ ] Feature extraction (lines, blobs, corners)
- [ ] Data association for SLAM and localization

### Phase 3: Sensor Fusion (not started)

- [ ] Extended Kalman Filter (EKF)
- [ ] Unscented Kalman Filter (UKF)
- [ ] Performance comparison

### Phase 4: Localization (not started)

- [ ] Iterative Closest Point
- [ ] Kalman filters with iterative solution
- [ ] Monte Carlo Localization

### Phase 5: SLAM (not started)

- [ ] Occupancy grid mapping
- [ ] EKF SLAM with unknown correspondences

### Phase 6: Path Planning (not started)

## Troubleshooting

### RViz model blinks/flashes

**Cause:** Multiple `/clock` publishers (e.g., a stale `clock_bridge` from a previous session). Two clock sources interleave messages via DDS, causing sub-millisecond backward time jumps. tf2 clears its buffer on any backward jump → RViz loses transforms → blink.

**Fix:**
```bash
# Check for multiple /clock publishers
ros2 topic info /clock --verbose

# Kill any stale bridge processes
ps aux | grep parameter_bridge | grep -v grep
kill <stale_pid>
```

### "Moved backwards in time" warnings

**Cause:** Nodes with `use_sim_time=true` started before `/clock` was available, so they initialized on wall clock time. When sim clock arrives (~0 seconds), it's a massive backward jump.

**Fix:** The launch file uses `TimerAction(5s)` to delay sim-time nodes. If warnings persist, increase the delay or ensure Gazebo fully starts before launching nodes.

### Robot not visible in Gazebo

**Cause:** Mesh URIs use `package://robot_description/meshes/...` which gets converted to `model://robot_description/meshes/...`. Gazebo needs the parent of the package share directory in `GZ_SIM_RESOURCE_PATH`.

**Fix:** The launch file sets this automatically via `AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ...)`. If meshes still don't load, check that the package is built and installed.
