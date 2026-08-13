# 6. Multi-Sensor Cache with Callback Groups

When you have multiple independent sensor subscriptions feeding one output
topic, use a shared struct and a single mutex:

```
/scan_status  ──┐
/imu_status   ──┼──► [Cache struct] ──► [Timer 200ms] ──► /pipeline/summary
/odom_status  ──┘       (mutex)              (snapshot)
```

```cpp
struct Cache {
    std::optional<std::string> scan, imu, odom;
    std::mutex mutex;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node  = std::make_shared<rclcpp::Node>("sensor_pipeline");
    auto cache = std::make_shared<Cache>();

    // One MutuallyExclusive group for all subscribers
    auto group = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = group;

    // Helper to create a subscriber with a field setter
    auto make_sub = [&](const std::string& topic, auto setter) {
        return node->create_subscription<std_msgs::msg::String>(
            topic, 10,
            [cache, setter](const std_msgs::msg::String::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(cache->mutex);
                setter(*cache, msg->data);
            },
            opts);
    };

    auto scan = make_sub("/scan_status",
        [](Cache& c, const std::string& v) { c.scan = v; });
    auto imu  = make_sub("/imu_status",
        [](Cache& c, const std::string& v) { c.imu  = v; });
    auto odom = make_sub("/odom_status",
        [](Cache& c, const std::string& v) { c.odom = v; });

    auto pub = node->create_publisher<std_msgs::msg::String>("/pipeline/summary", 10);

    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(200),
        [cache, pub] {
            // Snapshot under lock
            std::optional<std::string> s, i, o;
            {
                std::lock_guard<std::mutex> lock(cache->mutex);
                s = cache->scan; i = cache->imu; o = cache->odom;
            }
            std_msgs::msg::String msg;
            msg.data = (s && i && o) ? "OK" : "MISSING_INPUTS";
            pub->publish(msg);
        });

    (void)scan; (void)imu; (void)odom;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

> **Production trade-off:** With `SingleThreadedExecutor`, the MutuallyExclusive
> group and the mutex are both redundant — only one callback runs at a time.
> But keeping both means the code is correct if you later switch executors.
> Cost: near-zero. Benefit: safe refactor later.
