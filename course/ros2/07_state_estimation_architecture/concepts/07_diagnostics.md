# 6. Diagnostics

Diagnostics tell operators what the estimator is doing: how many measurements
were accepted/rejected, whether the filter is initialised, and when it last
received a useful update.

```cpp
std::string diagnostics(const rclcpp::Clock& clock) const {
    std::ostringstream out;
    out << "initialized=" << (initialized_ ? "true" : "false")
        << " accepted=" << accepted_
        << " rejected=" << rejected_
        << " state=(" << x_ << "," << y_ << ")"
        << " predict_age_s=" << (clock.now() - last_predict_stamp_).seconds()
        << " update_age_s="  << (clock.now() - last_update_stamp_).seconds();
    return out.str();
}
```

Publish on a timer so monitoring tools can read it independently:

```cpp
auto diag_pub = node->create_publisher<std_msgs::msg::String>(
    "localization/status", 10);

auto diag_timer = node->create_wall_timer(500ms, [estimator, diag_pub, node] {
    std_msgs::msg::String msg;
    msg.data = estimator->diagnostics(*node->get_clock());
    diag_pub->publish(msg);
});
```
