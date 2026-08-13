# 8. Common Mistakes

### Mistake 1 — Blocking inside a callback

```cpp
auto sub = node->create_subscription<std_msgs::msg::String>(
    "/cmd", 10, [](const std_msgs::msg::String::SharedPtr msg) {
        std::this_thread::sleep_for(std::chrono::seconds(1));  // BLOCKS executor
    });
```

One blocked callback stalls every other callback sharing that executor thread.

**Fix:** Do heavy work in a separate std::thread or std::async. Post the result
back via a thread-safe queue read by a timer callback.

### Mistake 2 — Not protecting shared state

```cpp
// timer and subscription both write latest_ without a mutex — DATA RACE
std::string latest_;
auto sub = node->create_subscription<...>("/topic", 10,
    [this](const std_msgs::msg::String::SharedPtr m) { latest_ = m->data; });
auto timer = node->create_wall_timer(100ms,
    [this] { RCLCPP_INFO(get_logger(), latest_); });  // race!
```

Even with `SingleThreadedExecutor` this is *currently* safe, but becomes a race
the moment you add a `MultiThreadedExecutor`. Write it correctly from the start.

### Mistake 3 — Holding a lock while publishing

```cpp
// WRONG
std::lock_guard<std::mutex> lock(mutex_);
pub_->publish(msg);   // publish acquires internal DDS locks — may deadlock
```

Always release your mutex before calling `publish()`.
