# ROS2 Module 10 — Gazebo Integration

## Goal

Use Gazebo Harmonic as an integration environment while keeping simulation-specific
wiring completely separate from production node logic. A node that monitors sensor
health should run unchanged on hardware and in simulation — only the launch file
and bridge configuration differ.

> **See also:** https://gazebosim.org/docs/harmonic/ros2_integration
> for the official ros_gz_bridge setup guide. This module focuses on the
> architectural discipline needed to keep simulation wiring out of business logic.

---

## 1. What Gazebo Provides

Gazebo Harmonic runs a physics simulation with:
- A robot model (URDF/SDF) — joints, links, inertia, collision geometry
- Sensor plugins (GPU LiDAR, IMU, odometry, GPS)
- Actuator plugins (differential drive, joint controllers)
- A simulation clock published on `/clock`

ROS2 nodes see none of this directly — they see ROS2 topics bridged from Gazebo.

```
Gazebo Harmonic
    │
    │ Gazebo transport (gz::msgs)
    │
ros_gz_bridge
    │
    │ ROS2 DDS
    │
Your node (same code as on hardware)
```

---

## 2. Simulation Time

### The `/clock` topic

Gazebo publishes `rosgraph_msgs/Clock` on `/clock`. Every node with
`use_sim_time: true` uses this as its time source instead of the OS clock.

```cpp
// With use_sim_time: true in launch file:
auto now = node->now();   // returns Gazebo simulation time
```

### Startup ordering problem

Nodes start before Gazebo emits any clock messages. A node with `use_sim_time`
will hang in `node->now()` until the first `/clock` message arrives.

```
t=0s: ros_gz_bridge starts → /clock not yet published
t=2s: your_node starts, calls node->now() → BLOCKS
t=3s: Gazebo ready, /clock starts → node unblocks
```

The CLAUDE.md `TimerAction(5s)` in `gazebo.launch.py` handles this: the bridge
starts immediately to establish `/clock`, and all other nodes start 5 s later.

### Checking the clock source at runtime

```cpp
auto timer = node->create_wall_timer(500ms, [node, pub] {
    const auto now = node->now();
    std::ostringstream s;
    s << "use_sim_time="
      << (node->get_parameter("use_sim_time").as_bool() ? "true" : "false")
      << " now_s=" << now.seconds()
      << " clock_type=" << now.get_clock_type();
    std_msgs::msg::String msg;
    msg.data = s.str();
    pub->publish(msg);
});
```

`clock_type` returns `3` for `RCL_ROS_TIME` (sim time) and `1` for
`RCL_SYSTEM_TIME` (wall time). A mismatch between publisher and subscriber
clock types causes timestamp arithmetic to fail.

---

## 3. The Bridge Configuration

`ros_gz_bridge` maps Gazebo topic names to ROS2 topic names. Bridge config is
a YAML file — it is integration wiring, not business logic:

```yaml
# bridge.yaml
- topic_name: /world/default/model/robot/.../front_laser/scan
  ros_topic_name: /front_laser/scan
  gz_type_name: gz.msgs.LaserScan
  ros_type_name: sensor_msgs/msg/LaserScan
  direction: GZ_TO_ROS

- topic_name: /model/robot/cmd_vel
  ros_topic_name: /cmd_vel
  gz_type_name: gz.msgs.Twist
  ros_type_name: geometry_msgs/msg/Twist
  direction: ROS_TO_GZ
```

```python
# In launch file
Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{'config_file': bridge_config_path}],
)
```

---

## 4. Gazebo Monitor Pattern

A monitoring node that watches the Gazebo clock: if the clock goes silent for
more than a grace period, report `WAITING_FOR_GAZEBO_CLOCK`:

```cpp
#include "rosgraph_msgs/msg/clock.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("gazebo_monitor");
    node->declare_parameter<double>("startup_grace_s", 5.0);

    // Wall-time baseline — used to compute age without blocking on sim clock
    auto first_wall   = std::make_shared<rclcpp::Time>(node->get_clock()->now());
    auto last_clock   = std::make_shared<rclcpp::Time>(0, 0, RCL_ROS_TIME);
    auto clock_count  = std::make_shared<int>(0);

    // Subscribe to /clock directly — not via node->now()
    auto clock_sub = node->create_subscription<rosgraph_msgs::msg::Clock>(
        "clock", 10,
        [last_clock, clock_count](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
            *last_clock = msg->clock;
            ++(*clock_count);
        });

    auto pub = node->create_publisher<std_msgs::msg::String>("gazebo/monitor_status", 10);

    auto timer = node->create_wall_timer(500ms,
        [node, pub, first_wall, last_clock, clock_count] {
            const double grace_s   = node->get_parameter("startup_grace_s").as_double();
            const double wall_age  = (node->get_clock()->now() - *first_wall).seconds();
            const bool in_grace    = wall_age < grace_s;
            const bool clock_seen  = *clock_count > 0;

            std::ostringstream s;
            s << "state=" << ((clock_seen || in_grace) ? "OK" : "WAITING_FOR_GAZEBO_CLOCK")
              << " clock_messages=" << *clock_count
              << " last_clock_s=" << last_clock->seconds()
              << " grace_s=" << grace_s;
            std_msgs::msg::String msg;
            msg.data = s.str();
            pub->publish(msg);
        });

    (void)clock_sub; (void)timer;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

Note: The timer uses `node->get_clock()->now()` not `node->now()`. When
`use_sim_time` is false (common for monitoring nodes that need wall time
regardless), the two are equivalent. When `use_sim_time` is true, `node->now()`
returns sim time — which may be zero before the clock starts.

---

## 5. Data Flow in This Repository

```
keyboard/nav stack
   │ /cmd_vel (Twist)
   ▼
ros_gz_bridge ──────────────────► Gazebo Harmonic (gz-sim)
                                    DiffDrive plugin → wheel joints, /odom
                                    gpu_lidar (2D) → /front_laser/scan
                                    gpu_lidar (3D) → /top_lidar_3d/points
                                    IMU / GPS / Magnetometer
ros_gz_bridge ◄─── all gz topics ──┘
robot_state_publisher ◄─ /joint_states (ros) ─ joint_state_publisher (ROS timer)
rviz2 ◄─ TF tree, sensor topics
```

Key design decision: `/tf` and `/joint_states` are NOT bridged from Gazebo.
Instead, `joint_state_publisher` (a ROS timer) is the sole source of
`/joint_states` — this prevents non-monotonic timestamps from causing the
`tf2` buffer to clear and RViz to blink.

---

## 6. Common Gazebo Bridge Issues

| Symptom | Cause | Fix |
|---|---|---|
| No messages on `/imu/data` | Bridge not running or topic name mismatch | `ros2 topic list` — check both sides |
| Timestamps at 0 | `use_sim_time` mismatch or clock not started | Check `ros2 param get /node use_sim_time` |
| Wrong `frame_id` | Bridge remapping doesn't match URDF frame | Match bridge config to URDF sensor frames |
| RViz model blinks | Multiple `/clock` publishers; non-monotonic tf | Check `ros2 topic info /clock --verbose`, kill stale bridges |
| "Moved backwards in time" | Node started before `/clock` | Use `TimerAction(5s)` delay in launch file |

### Debugging tools

```bash
# Check clock source and rate
ros2 topic hz /clock           # should be ~1000 Hz in Gazebo
ros2 topic info /clock --verbose  # shows all publishers

# Check what the bridge is exposing
ros2 topic list | grep -v /rosout | sort

# Check frame IDs on sensor topics
ros2 topic echo /imu/data --field header.frame_id --once
ros2 topic echo /front_laser/scan --field header.frame_id --once

# Check QoS compatibility
ros2 topic info /front_laser/scan --verbose
```

---

## 7. Writing Nodes Portable Between Sim and Hardware

The test: can you run the exact same binary against `ros2 topic pub` and
against Gazebo without any code change?

**Yes if:** The node reads sensor topics, uses `node->now()` for time, and
has `use_sim_time` injected by the launch file.

**No if:** The node checks for a Gazebo-specific topic name, forks based on
`use_sim_time`, or calls Gazebo APIs directly.

```cpp
// PORTABLE — topic name and QoS come from parameters / launch file
auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "imu_topic",  // parameterised in launch, not hardcoded
    rclcpp::QoS(10).best_effort(),
    callback);

// NOT PORTABLE — hardcoded Gazebo topic name
auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "/world/default/model/robot/link/imu_link/sensor/imu/imu",  // DO NOT DO THIS
    ...);
```

---

## 8. Production Trade-offs

| Decision | Trade-off |
|---|---|
| Bridge YAML vs bridge args | YAML: versionable, auditable. Args: quick for one-off debugging |
| `use_sim_time` for all nodes | All nodes advance together with Gazebo — required for correct timestamps |
| Startup delay vs readiness probe | `TimerAction(5s)` is simple but fragile if Gazebo is slow. Readiness probe is correct but more code |
| Monitoring clock directly vs via node->now() | Direct `/clock` sub works even before the node clock is synchronised |
| Separate sim launch vs `IfCondition` | Separate: clear ownership. IfCondition: one file but more logic |

---

## 9. Checklist

- [ ] `use_sim_time: true` set for all nodes in simulation launch file
- [ ] Bridge started first — before nodes that consume sensor topics
- [ ] Nodes don't subscribe to Gazebo transport topic names directly
- [ ] `frame_id` in sensor messages matches URDF sensor frames
- [ ] Startup delay handles clock-before-nodes ordering
- [ ] Monitoring node reports `WAITING_FOR_GAZEBO_CLOCK` during startup grace period
- [ ] All topic names that differ between sim and hardware are launch arguments, not hardcoded
