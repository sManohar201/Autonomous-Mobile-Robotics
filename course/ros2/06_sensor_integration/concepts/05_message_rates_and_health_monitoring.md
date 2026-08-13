# 4. Message Rates and Health Monitoring

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
