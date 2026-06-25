# ROS2 Module 06 — Sensor Integration

## Goal

Subscribe to real or simulated robot sensor streams correctly: verify frame IDs,
check timestamps, monitor publication rates, detect stale inputs, and publish a
health verdict. These patterns appear in every production sensor fusion pipeline.

> **See also:** https://docs.ros.org/en/jazzy/Concepts/Basic/About-ROS-Interfaces.html
> for the message type catalogue. Sensor-specific message definitions live in
> `sensor_msgs`, `nav_msgs`, and `geometry_msgs`.

---

## 1. Common Sensor Topics

### Message types and frames

| Topic | Message type | Typical frame | Typical rate |
|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | `imu_link` | 100 Hz |
| `/odom` | `nav_msgs/Odometry` | `odom` → `base_link` | 50 Hz |
| `/front_laser/scan` | `sensor_msgs/LaserScan` | `laser_link` | 10–50 Hz |
| `/top_lidar_3d/points` | `sensor_msgs/PointCloud2` | `lidar_link` | 10 Hz |
| `/gps/fix` | `sensor_msgs/NavSatFix` | `gps_link` | 10 Hz |
| `/magnetometer` | `sensor_msgs/MagneticField` | `imu_link` | 50 Hz |

The topic name and message type are the interface contract. The frame ID and
timestamp inside the message are the data contract. Both must be correct.

### Include paths

```cpp
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
```

In `package.xml`:
```xml
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
<depend>geometry_msgs</depend>
```

---

## 2. Timestamps

### What `header.stamp` means

Every stamped sensor message contains:
```
std_msgs/Header header
  builtin_interfaces/Time stamp   ← when the measurement was taken
  string frame_id                 ← coordinate frame
```

`stamp` is **measurement time**, not receive time. The driver sets it to the
hardware trigger time (or the best available estimate). It tells downstream
consumers where in time this sample belongs.

```cpp
auto sub = node->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10,
    [node](const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Age of sample at receive time
        auto age = node->now() - msg->header.stamp;
        RCLCPP_DEBUG(node->get_logger(), "IMU age: %.1f ms",
                     age.seconds() * 1e3);
    });
```

### Wall time vs sim time

| Mode | `node->now()` returns | When to use |
|---|---|---|
| Wall time (default) | OS real time | Real robot |
| Sim time (`use_sim_time: true`) | `/clock` topic time | Gazebo simulation |

With sim time enabled, `node->now()` blocks until the first `/clock` message
arrives. If the bridge is not running, the node hangs at startup.

```python
# launch file — enable sim time for simulation nodes
Node(
    package='my_pkg',
    executable='my_node',
    parameters=[{'use_sim_time': True}]
)
```

> **Production trade-off:** Never hardcode `rclcpp::Clock(RCL_SYSTEM_TIME)` in
> a node that may run in simulation. Always use `node->get_clock()` or
> `node->now()` — the correct clock source is injected by the runtime.

### Detecting stale data

A message is "stale" if its stamp is older than a threshold relative to now.
The threshold depends on the sensor rate:

```
IMU at 100 Hz → expect a sample every 10 ms → stale after 100 ms
LiDAR at 10 Hz → expect a sample every 100 ms → stale after 1 s
```

```cpp
bool is_stale(const rclcpp::Time& stamp, const rclcpp::Node::SharedPtr& node,
              double threshold_seconds) {
    return (node->now() - stamp).seconds() > threshold_seconds;
}
```

> **Caution:** `node->now() - msg->header.stamp` can throw if the two `Time`
> objects have different clock types (RCL_SYSTEM_TIME vs RCL_ROS_TIME). Use
> `rclcpp::Duration` with the node clock to stay consistent.

---

## 3. Frame IDs and TF2

### Why frame IDs matter

A sensor measurement without a frame ID is a number without units — useless to
any consumer that needs to relate it to robot pose.

```
/imu/data.header.frame_id = "imu_link"
               means:
"this linear acceleration vector is expressed in the imu_link frame"
```

A consumer wanting IMU data in `base_link` coordinates must look up the
`imu_link` → `base_link` transform from TF2.

### Validating frame IDs

The simplest check: compare the actual `frame_id` to the expected value.

```cpp
auto expected = node->declare_parameter<std::string>("expected_frame_id", "imu_link");
auto sub = node->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10,
    [pub, expected](const sensor_msgs::msg::Imu::SharedPtr msg) {
        std_msgs::msg::String status;
        status.data = (msg->header.frame_id == expected)
            ? "OK"
            : "FRAME_ERROR expected=" + expected +
              " actual=" + msg->header.frame_id;
        pub->publish(status);
    });
```

Make `expected_frame_id` a parameter so the same node works for any sensor
without recompilation.

### The TF2 tree

```
map
 └── odom
      └── base_link
           ├── imu_link
           ├── laser_link
           ├── lidar_link
           └── gps_link
```

Each edge is a transform published by a TF2 broadcaster. The robot description
(URDF/xacro) defines the static edges. Dynamic edges (odom → base_link) are
published by wheel odometry or a filter.

```cpp
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

auto tf_buffer    = std::make_shared<tf2_ros::Buffer>(node->get_clock());
auto tf_listener  = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

// Look up imu_link pose in base_link at a specific time
auto tf = tf_buffer->lookupTransform("base_link", "imu_link",
                                      tf2::TimePointZero);
```

> **See also:** https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html

---

## 4. Message Rates and Health Monitoring

### Why rate monitoring matters

A topic that exists but is silent is as bad as one that doesn't exist.
Production robots must distinguish:

- **Good:** LiDAR topic present, publishing at 10 Hz, frame ID correct
- **Degraded:** LiDAR topic present but rate dropped to 2 Hz (cable issue)
- **Failed:** LiDAR topic present, no messages for > 1 s

### Rate tracking pattern

Track the last time a message was received. A timer checks periodically whether
any sensor has gone silent:

```cpp
struct Seen {
    rclcpp::Time stamp;
    bool received{false};
    std::string frame;
};
```

On each subscription callback:
```cpp
auto mark = [node](std::shared_ptr<Seen> seen, const std::string& frame_id) {
    seen->stamp    = node->now();
    seen->received = true;
    seen->frame    = frame_id;
};
```

On each timer callback:
```cpp
auto stale = [&](const std::shared_ptr<Seen>& s) {
    return !s->received ||
           (node->now() - s->stamp) > rclcpp::Duration::from_seconds(1.0);
};
bool healthy = !stale(imu) && !stale(odom) && !stale(scan);
```

### Full sensor health node

```cpp
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

struct Seen {
    rclcpp::Time stamp;
    bool received{false};
    std::string frame;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sensor_health");

    auto imu_seen  = std::make_shared<Seen>();
    auto odom_seen = std::make_shared<Seen>();
    auto scan_seen = std::make_shared<Seen>();

    auto mark = [node](std::shared_ptr<Seen> seen, const std::string& frame) {
        seen->stamp    = node->now();
        seen->received = true;
        seen->frame    = frame;
    };

    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10,
        [=](const sensor_msgs::msg::Imu::SharedPtr msg) {
            mark(imu_seen, msg->header.frame_id);
        });

    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        [=](const nav_msgs::msg::Odometry::SharedPtr msg) {
            mark(odom_seen, msg->header.frame_id);
        });

    auto scan_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/front_laser/scan", 10,
        [=](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            mark(scan_seen, msg->header.frame_id);
        });

    auto pub = node->create_publisher<std_msgs::msg::String>("/sensor/health", 10);

    auto timer = node->create_wall_timer(500ms, [=, node] {
        auto stale = [&](const std::shared_ptr<Seen>& s) {
            return !s->received ||
                   (node->now() - s->stamp) > rclcpp::Duration::from_seconds(1.0);
        };
        std_msgs::msg::String out;
        out.data = (!stale(imu_seen) && !stale(odom_seen) && !stale(scan_seen))
                   ? "OK" : "STALE_INPUTS";
        pub->publish(out);
    });

    (void)imu_sub; (void)odom_sub; (void)scan_sub;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

---

## 5. QoS for Sensor Topics

Different sensors have different reliability requirements:

| Sensor stream | Reliability | Durability | Depth | Why |
|---|---|---|---|---|
| IMU (100 Hz) | `best_effort` | `volatile` | 5–10 | High rate; old samples are worthless |
| Odometry (50 Hz) | `best_effort` | `volatile` | 5 | Same: latency matters more than delivery |
| LiDAR scan (10 Hz) | `reliable` | `volatile` | 5 | Lower rate; reliability worthwhile |
| GPS fix (1–10 Hz) | `reliable` | `transient_local` | 1 | Late-joining nodes need last known fix |
| Camera image (30 Hz) | `best_effort` | `volatile` | 1 | Bandwidth-dominated; drop is acceptable |

```cpp
// Best-effort subscriber for high-rate IMU
auto qos = rclcpp::QoS(10).best_effort();
auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", qos, callback);

// Reliable subscriber for GPS
auto gps_qos = rclcpp::QoS(1).reliable().transient_local();
auto gps_sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/gps/fix", gps_qos, callback);
```

> **Production trade-off:** If a subscriber uses `reliable` but the publisher
> uses `best_effort`, DDS won't establish the connection — the subscription
> silently receives nothing. Match QoS or explicitly use `keep_last(1)` with
> compatible profiles. Use `ros2 topic info -v /topic_name` to inspect QoS on
> both ends.

---

## 6. Covariance Fields

Many sensor messages carry a covariance matrix expressing measurement uncertainty.
State estimation filters (EKF, UKF) need these values.

### IMU covariance

```cpp
sensor_msgs::msg::Imu msg;
// 3x3 covariance for orientation (row-major)
msg.orientation_covariance[0] = 0.01;   // var(roll)
msg.orientation_covariance[4] = 0.01;   // var(pitch)
msg.orientation_covariance[8] = 0.02;   // var(yaw) — heading noisier
// -1 means "this field is not provided"
msg.linear_acceleration_covariance[0] = -1.0;
```

### Odometry covariance

```cpp
nav_msgs::msg::Odometry odom;
// 6x6 pose covariance [x,y,z,roll,pitch,yaw] (row-major, 36 elements)
odom.pose.covariance[0]  = 0.001;   // var(x)
odom.pose.covariance[7]  = 0.001;   // var(y)
odom.pose.covariance[35] = 0.01;    // var(yaw)
// Twist covariance similarly
odom.twist.covariance[0]  = 0.001;  // var(vx)
odom.twist.covariance[35] = 0.005;  // var(vyaw)
```

> **Interview question:** "A subscriber receives an IMU message with
> `angular_velocity_covariance[0] == -1`. What does that mean?"  
> **Answer:** The field is not provided. Consumers should fall back to a default
> noise model or ignore that component.

---

## 7. Sensor Integration Patterns

### Pattern A — Frame validator

Validate every incoming message has the expected frame ID. Parameterize so the
same binary handles multiple sensors:

```
                       ┌─────────────────────┐
/imu/data ────────────►│  frame_validator     │──► /sensor/frame_status
                       │  (expected_frame_id  │
                       │   param: "imu_link") │
                       └─────────────────────┘
```

### Pattern B — Multi-sensor health monitor

```
/imu/data          ──┐
/odom              ──┼──► [Seen structs] ──► [Timer 500ms] ──► /sensor/health
/front_laser/scan  ──┘    (last seen time)    (stale check)
```

### Pattern C — Rate estimator

Track sample count over a sliding window to compute actual Hz:

```cpp
struct RateEstimator {
    rclcpp::Time window_start;
    int count{0};
    double hz{0.0};

    void mark(const rclcpp::Time& now) {
        ++count;
        double elapsed = (now - window_start).seconds();
        if (elapsed > 1.0) {
            hz = count / elapsed;
            count = 0;
            window_start = now;
        }
    }
};
```

---

## 8. Gazebo Sensor Bridge

In simulation, sensor data originates from Gazebo plugins and is bridged to ROS2
topics by `ros_gz_bridge`. The bridge runs as a separate process and maps
Gazebo topic names to ROS2 topic names.

```
Gazebo plugin (gpu_lidar)
    │ /world/default/model/robot/link/laser_link/sensor/front_laser/scan [gz::msgs::LaserScan]
    ▼
ros_gz_bridge
    │ /front_laser/scan [sensor_msgs/LaserScan]
    ▼
Your ROS2 node
```

The bridge handles timestamp conversion: Gazebo wall time → ROS2 `header.stamp`.
With `use_sim_time: true`, timestamps come from the Gazebo `/clock` topic.

### Verifying sensor topics in simulation

```bash
# List all active topics
ros2 topic list

# Check type and rate
ros2 topic info /front_laser/scan --verbose
ros2 topic hz /imu/data

# Inspect a message
ros2 topic echo /odom --once
```

### Common bridge issues

| Symptom | Cause | Fix |
|---|---|---|
| No messages on `/imu/data` | Bridge not running or topic name mismatch | Check `ros2 topic list` vs Gazebo topic names |
| Timestamps at time 0 | `use_sim_time` mismatch | Ensure all nodes have matching clock source |
| Wrong frame_id | URDF frame != bridge remapping | Match URDF sensor frame names to plugin config |

---

## 9. Testing Sensor Integration without Hardware

Use `ros2 topic pub` to inject synthetic sensor data:

```bash
# Publish a synthetic IMU message
ros2 topic pub /imu/data sensor_msgs/msg/Imu \
  '{header: {frame_id: imu_link}, angular_velocity: {z: 0.1}}' \
  --rate 100

# Publish with wrong frame to test frame_validator
ros2 topic pub /imu/data sensor_msgs/msg/Imu \
  '{header: {frame_id: wrong_frame}}' \
  --rate 10
```

For automated tests, use `rclcpp::executors::SingleThreadedExecutor` with a
`WallTimer` that publishes test messages and another timer that reads the output.
See Module 09 (Testing & CI) for the full pattern.

---

## 10. Production Trade-offs

| Decision | Trade-off |
|---|---|
| `best_effort` vs `reliable` QoS | Best-effort: no buffering, minimal latency. Reliable: retries, adds latency and memory |
| Stale threshold | Too tight: false positives in bursty networks. Too loose: real failures slip through |
| Frame validation at every callback | Correct but adds per-message CPU. Alternative: validate once at startup via service |
| Separate health node vs inline | Separate: testable, reusable. Inline: simpler, fewer processes |
| `node->now()` vs `msg->header.stamp` for stale check | Use `node->now()` for receive-time staleness; `msg->header.stamp` for data-freshness (lag from sensor) |

---

## 11. Checklist for a Sensor Integration Node

- [ ] QoS profile matches publisher — verified with `ros2 topic info -v`
- [ ] Frame ID validation is parameterised, not hardcoded
- [ ] Stale detection threshold is proportional to expected sensor rate
- [ ] Node tested under both wall time and sim time
- [ ] Health status published at a rate independently readable by operators
- [ ] Covariance fields checked — nodes don't silently use `covariance[0] == -1`
