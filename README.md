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
# Default empty world
ros2 launch robot_description gazebo.launch.py

# Specific world
ros2 launch robot_description gazebo.launch.py world:=slam_district
```

This starts:
1. **Gazebo Harmonic** — physics simulation with DiffDrive, sensor, and joint state plugins
2. **ros_gz_bridge** — bridges all sensor and control topics between Gazebo and ROS2
3. **robot_state_publisher** — publishes the TF tree from URDF + joint states
4. **joint_state_publisher** — publishes joint positions (sole source, monotonic timestamps)
5. **RViz2** — visualization with robot model, laser scans, point cloud, and path displays

To drive the robot, open a second terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### World Selection

```bash
ros2 launch robot_description gazebo.launch.py world:=<name>
```

| World | Description |
|-------|-------------|
| `empty` | Flat ground plane (default) |
| `slam_district` | Large 5-zone world for persistent mapping / loop-closure testing |
| `hospital` | Indoor hospital with moving carts |
| `warehouse` | Large warehouse with patrolling forklifts |
| `maze` | Tight corridor maze |
| `office` | Clearpath indoor office environment |
| `outdoor_urban` | Urban street scene with pedestrians |

A full path to any `.sdf` file is also accepted:
```bash
ros2 launch robot_description gazebo.launch.py world:=/path/to/my_world.sdf
```

### Model Viewer Only (no simulation)

```bash
ros2 launch robot_description description.launch.py
```

Opens RViz2 with `joint_state_publisher_gui` sliders — useful for inspecting the URDF without Gazebo.

## Architecture

```
                                          ┌─────────────────────┐
keyboard / nav stack                      │   Gazebo Harmonic    │
     │                                    │   (gz-sim)           │
     │ /cmd_vel (ROS2 Twist)              │                      │
     ▼                                    │   DiffDrive plugin   │
ros_gz_bridge ──────────────────────────► │   → wheel joints     │
                                          │   → /odom (gz)       │
                    /odom (Odometry)       │                      │
robot_state_pub  ◄──────────────────────  │   gpu_lidar (2D)     │
rviz2            ◄─  ros_gz_bridge  ◄───  │   → /front_laser/scan│
                    /front_laser/scan      │                      │
                    /top_lidar_3d/points   │   gpu_lidar (3D)     │
                    /imu/data              │   → /top_lidar_3d    │
                    /gps/fix               │                      │
                    /magnetometer          │   IMU / GPS / Mag    │
                    /clock                 │                      │
                                          │   JointStatePub      │
                                          │   → /joint_states    │
                                          └─────────────────────┘
```

### Topic Bridge Map (Gazebo ↔ ROS2)

| Topic | Message Type | Direction | Notes |
|-------|-------------|-----------|-------|
| `/clock` | `Clock` | gz → ros | |
| `/cmd_vel` | `Twist` | ros → gz | |
| `/odom` | `Odometry` | gz → ros | from DiffDrive plugin |
| `/front_laser/scan` | `LaserScan` | gz → ros | 2D LiDAR, 270° FOV |
| `/top_lidar_3d/points` | `PointCloud2` | gz → ros | 3D LiDAR, VLP-16 style |
| `/imu/data` | `Imu` | gz → ros | 100 Hz |
| `/magnetometer` | `MagneticField` | gz → ros | 50 Hz |
| `/gps/fix` | `NavSatFix` | gz → ros | 10 Hz |

**Not bridged (intentionally):**

| Topic | Reason |
|-------|--------|
| `/tf` | DiffDrive publishes `odom→base_link` TF with Gazebo timestamps. DDS delivers these out of order → non-monotonic timestamps → tf2 buffer clears → RViz blinks. Will be replaced by `diff_drive_controller` once `ros2_control` is wired up. |
| `/joint_states` | Same DDS ordering issue. `joint_state_publisher` (ROS timer) is the sole source — always strictly monotonic. |

### Startup Sequencing

ROS nodes with `use_sim_time=true` start on wall clock (~1.7 billion seconds). Gazebo sim time starts at ~0. If nodes start before `/clock` is established, they see a massive backward time jump.

**Solution:** `TimerAction(5s)` delays `robot_state_publisher`, `joint_state_publisher`, `spawn_robot`, and `rviz2`. The bridge starts immediately to deliver `/clock` first.

## Robot Sensors

| Sensor | Link | Topic | Rate | Notes |
|--------|------|-------|------|-------|
| 2D LiDAR (Hokuyo UST-10) | `front_laser` | `/front_laser/scan` | 50 Hz | Front-centre, z=0.19 m, 270° FOV, 30 m range |
| 3D LiDAR (VLP-16 style) | `top_lidar_3d` | `/top_lidar_3d/points` | 10 Hz | Top-centre, z=0.32 m, ±15° vertical, 100 m range |
| IMU | `imu_link` | `/imu/data` | 100 Hz | MEMS noise + bias model |
| GPS / NavSat | `gps_link` | `/gps/fix` | 10 Hz | 1 m horizontal noise |
| Magnetometer | `imu_link` | `/magnetometer` | 50 Hz | σ = 1.3 × 10⁻⁴ T |

## Package Structure

```
src/robot_description/
├── urdf/
│   ├── automaton.urdf.xacro    # Main robot model (chassis, wheels, sensor mounts)
│   ├── automaton.gazebo        # Gazebo plugins (DiffDrive, IMU, GPS, Magnetometer)
│   ├── laser.urdf.xacro        # 2D LiDAR macro (Hokuyo UST-10, gpu_lidar)
│   └── lidar3d.urdf.xacro      # 3D LiDAR macro (VLP-16 style, gpu_lidar)
├── meshes/
│   ├── base.stl                # Chassis mesh
│   ├── wheel.stl               # Wheel mesh
│   ├── hokuyo_ust10_lidar.stl  # 2D LiDAR mesh
│   └── d435.dae                # Depth camera mesh (unused)
├── worlds/
│   ├── automaton_world.sdf     # Default empty world
│   ├── slam_district.sdf       # Large 5-zone SLAM benchmark world
│   ├── hospital.sdf            # Indoor hospital with moving carts
│   ├── warehouse.sdf           # Warehouse with patrolling forklifts
│   ├── maze.sdf                # Tight corridor maze
│   ├── office.sdf              # Indoor office environment
│   └── outdoor_urban.sdf       # Urban street with pedestrians
├── launch/
│   ├── gazebo.launch.py        # Full simulation launch (world:= arg)
│   └── description.launch.py   # Standalone model viewer
├── rviz_config/
│   └── automaton.rviz          # RViz2 config (LaserScan + PointCloud2 + paths)
├── package.xml
└── CMakeLists.txt
```

## slam_district World

A purpose-built large-scale world for persistent mapping and loop-closure stress-testing.

```
              Zone 1 — Pillar Forest  (dense, varied heights)
                   y ∈ [45, 125]
                        │ N-arch
    Zone 5              │                Zone 2
    Sculptural Park ── HUB ─────────── Industrial Block
    x ∈ [-130,-30]      │                x ∈ [30, 125]
    (sparse, unique)    │ S-arch          (corridor run)
                   Zone 4 — Residential Grid
                   y ∈ [-55, -135]  ← aliasing zone
```

| Element | Purpose |
|---------|---------|
| 25 m red tower + cross fins | Global bearing anchor, visible from all zones |
| Colour-coded gateway arches | Unambiguous zone-transition markers |
| Zone 1: 20-pillar grid (2/4/6 m heights) | High feature density, fast scan-match re-entry |
| Zone 2: parallel warehouses + 6 m corridor | Corridor ambiguity stress-test |
| Zone 2: orange midpoint cylinder | Breaks corridor symmetry (loop closure trigger) |
| Zone 4: 9 identical 10×8×3 m houses | Perceptual aliasing → false-positive LC rejection test |
| Zone 4: sphere entry markers + buttress wall | Zone-boundary disambiguation cues |
| Zone 5: monolith, arch, wedge sculptures | Unique geometry under sparse-feature conditions |

**Suggested routes:**
- Inner loop (~550 m, ~9 min at 1 m/s): visit each zone entry and return through hub
- Full perimeter (~800 m, ~13 min): traverse all 5 zones in sequence

## Port Progress

### Phase 1: Robot Description & Simulation ✅

- [x] `package.xml` → ament_cmake format 3
- [x] `CMakeLists.txt` → ament_cmake
- [x] Gazebo plugins → Harmonic API (`gz-sim-diff-drive-system`, `gz-sim-joint-state-publisher-system`)
- [x] 2D LiDAR (Hokuyo UST-10) → `gpu_lidar` at front-centre, z=0.19 m
- [x] 3D LiDAR (VLP-16 style) → `gpu_lidar` on top-centre, z=0.32 m, ±15° vertical, 360° horizontal
- [x] IMU → 100 Hz with MEMS noise + bias model
- [x] GPS / NavSat → 10 Hz, 1 m horizontal noise
- [x] Magnetometer → 50 Hz, σ = 1.3 × 10⁻⁴ T
- [x] World SDF with required system plugins (Sensors, Imu, NavSat, Magnetometer, Physics)
- [x] `spherical_coordinates` in all worlds (required for GPS and Magnetometer)
- [x] Launch files → ROS2 Python launch API with `world:=` argument
- [x] RViz config → LaserScan + PointCloud2 + EKF/Odom/GroundTruth path displays
- [x] ros_gz_bridge topic bridging with DDS timestamp fixes
- [x] Teleop keyboard control verified
- [x] 5 dynamic-obstacle worlds (hospital, warehouse, maze, office, outdoor_urban)
- [x] `slam_district` — large 5-zone SLAM benchmark world

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
