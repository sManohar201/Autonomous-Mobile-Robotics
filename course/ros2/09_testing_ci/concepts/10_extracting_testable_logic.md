# 9. Extracting Testable Logic

The most important testing pattern: **extract pure functions from node callbacks**.

```cpp
// Testable pure function
bool is_stale(const rclcpp::Time& now,
              const rclcpp::Time& last_seen,
              double timeout_s);

// Node thin wrapper — tests only that wiring is correct
auto timer = node->create_wall_timer(200ms, [node, last_seen, output] {
    const auto verdict = is_stale(node->now(), *last_seen,
                                  node->get_parameter("timeout_s").as_double());
    // ...publish verdict
});
```

If `is_stale` is tested by 5 unit tests, the node test only needs to verify
one case: that the output topic reflects the function's verdict. The unit tests
cover all the edge cases far more cheaply.
