# 2. Unit Tests — Pure C++ Logic

Logic that doesn't need a ROS2 node can be tested with standard C++ test
frameworks (GTest is the ROS2 standard).

### Freshness logic example

```cpp
bool is_stale(const rclcpp::Time& now,
              const rclcpp::Time& last_seen,
              double timeout_s) {
    if (last_seen.nanoseconds() == 0) {
        return true;  // never seen
    }
    return (now - last_seen).seconds() > timeout_s;
}
```

This function takes no `rclcpp::Node*` — it's a pure computation and can be
tested without spinning an executor:

```cpp
// GTest
TEST(FreshnessTest, NeverSeenIsStale) {
    rclcpp::Time now{1000000000LL, RCL_ROS_TIME};
    rclcpp::Time never{0, 0, RCL_ROS_TIME};
    EXPECT_TRUE(is_stale(now, never, 1.0));
}

TEST(FreshnessTest, FreshIsNotStale) {
    rclcpp::Time t0{1000000000LL, RCL_ROS_TIME};
    rclcpp::Time t1{1000500000LL, RCL_ROS_TIME};  // 0.5 s later
    EXPECT_FALSE(is_stale(t1, t0, 1.0));
}

TEST(FreshnessTest, StaleAfterTimeout) {
    rclcpp::Time t0{1000000000LL, RCL_ROS_TIME};
    rclcpp::Time t1{2000000000LL, RCL_ROS_TIME};  // 1 s later
    EXPECT_TRUE(is_stale(t1, t0, 0.5));
}
```

Key rule: if you need to call `rclcpp::init()` to test it, it's not a unit test.
Extract logic into free functions or classes that take clock values as arguments.
