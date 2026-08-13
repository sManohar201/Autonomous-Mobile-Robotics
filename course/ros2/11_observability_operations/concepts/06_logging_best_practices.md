# 5. Logging Best Practices

### Severity levels

```cpp
RCLCPP_DEBUG(logger, "Processing scan: %zu points", cloud.size());
RCLCPP_INFO(logger, "Localization initialized at (%.2f, %.2f)", x_, y_);
RCLCPP_WARN(logger, "Rejected measurement: %s", reason.c_str());
RCLCPP_ERROR(logger, "State estimator failed: no valid measurements in %.1f s", age);
RCLCPP_FATAL(logger, "Unsupported configuration: %s", config.c_str());
```

| Level | When to use |
|---|---|
| `DEBUG` | Internal state, high-frequency (rate-limited!) |
| `INFO` | State transitions, lifecycle events |
| `WARN` | Recoverable anomalies — operator should know |
| `ERROR` | Failed an operation — subsystem degraded |
| `FATAL` | Unrecoverable — node will exit |

### Rate limiting logs

```cpp
// At most one warning per 2 s — critical for 50 Hz sensor callbacks
RCLCPP_WARN_THROTTLE(
    node->get_logger(), *node->get_clock(), 2000,
    "Rejecting measurement: %s", reason.c_str());

// Log only on first occurrence
RCLCPP_WARN_ONCE(node->get_logger(), "First stale input detected");
```

Without throttling, a 50 Hz callback that logs every time it rejects fills
disk in minutes and buries useful signals in noise.
