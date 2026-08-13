# 3. Measurement Rejection

Filters should reject bad measurements rather than letting them corrupt state.
Three common rejection conditions:

| Condition | Check | Reason |
|---|---|---|
| Wrong frame | `frame_id != required_frame` | Data in wrong coordinate frame |
| High covariance | `cov_trace > threshold` | Sensor reporting low confidence |
| Outlier jump | `hypot(dx, dy) > max_jump` | Position jumped implausibly far |

```cpp
bool update(const nav_msgs::msg::Odometry& msg, std::string& reason) {
    if (msg.header.frame_id != required_frame_) {
        reason = "wrong_frame";
        ++rejected_;
        return false;
    }

    const double cov_trace = msg.pose.covariance[0]
                           + msg.pose.covariance[7]
                           + msg.pose.covariance[35];
    if (cov_trace > max_cov_trace_) {
        reason = "high_covariance";
        ++rejected_;
        return false;
    }

    const double dx = msg.pose.pose.position.x - x_;
    const double dy = msg.pose.pose.position.y - y_;
    if (initialized_ && std::hypot(dx, dy) > max_jump_m_) {
        reason = "outlier_jump";
        ++rejected_;
        return false;
    }

    // Accepted — update state
    x_ = msg.pose.pose.position.x;
    y_ = msg.pose.pose.position.y;
    initialized_ = true;
    ++accepted_;
    reason = "accepted";
    return true;
}
```

Log rejections with throttle to avoid log spam at 50 Hz:

```cpp
if (!accepted) {
    RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000,  // max 1 warning per 2 s
        "Rejected measurement: %s", reason.c_str());
}
```

> **Interview question:** "Your EKF keeps rejecting GPS updates. What's your
> investigation order?"
> 1. Check `frame_id` — wrong frame? Missing TF transform?
> 2. Check `covariance_trace` — sensor reporting very high uncertainty?
> 3. Check `outlier_jump` threshold — is the jump threshold too tight?
> 4. Look at the actual pose values — is GPS in a different datum or projection?
