# 5. Shared State and the Snapshot Pattern

The most common concurrency pattern in sensor fusion nodes:

```
         Subscription callback (fired by executor)
              │
              ▼
         [Lock mutex] → update shared cache → [Unlock]
         
         Timer callback (fired by executor)
              │
              ▼
         [Lock mutex] → copy snapshot → [Unlock]
              │
         (process snapshot without holding lock)
              │
         [publish result]
```

### Why copy-then-release?

If you hold the mutex during processing, the subscription callback blocks
waiting for the lock. For a 50 Hz LiDAR, a 20 ms processing stall drops a scan.

```cpp
// WRONG: holds lock during processing
auto timer = node->create_wall_timer(100ms, [cache, mutex, pub] {
    std::lock_guard<std::mutex> lock(*mutex);
    auto result = expensive_process(*cache);  // lock held here — blocks subs!
    pub->publish(result);
});

// CORRECT: snapshot pattern — lock only for the copy
auto timer = node->create_wall_timer(100ms, [cache, mutex, pub] {
    std::optional<std::string> snapshot;
    { std::lock_guard<std::mutex> lock(*mutex); snapshot = *cache; }  // brief lock
    // Process outside the lock — subscription callbacks can update cache now
    std_msgs::msg::String out;
    out.data = snapshot ? "processed " + *snapshot : "missing input";
    pub->publish(out);
});
```

### Full snapshot processor example

```cpp
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("snapshot_processor");

    // Shared state — accessed by subscription + timer callbacks
    auto latest = std::make_shared<std::optional<std::string>>();
    auto mutex  = std::make_shared<std::mutex>();

    auto pub = node->create_publisher<std_msgs::msg::String>("/processed/status", 10);

    // Subscription: write under lock
    auto sub = node->create_subscription<std_msgs::msg::String>(
        "/input/status", 10,
        [latest, mutex](const std_msgs::msg::String::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(*mutex);
            *latest = msg->data;
        });

    // Timer: copy snapshot, release lock, then publish
    auto timer = node->create_wall_timer(100ms, [latest, mutex, pub] {
        std::optional<std::string> snapshot;
        { std::lock_guard<std::mutex> lock(*mutex); snapshot = *latest; }

        std_msgs::msg::String out;
        out.data = snapshot ? "processed " + *snapshot : "missing input";
        pub->publish(out);
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
