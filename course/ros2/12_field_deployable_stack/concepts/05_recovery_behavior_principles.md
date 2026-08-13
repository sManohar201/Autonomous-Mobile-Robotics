# 4. Recovery Behavior Principles

Recovery must be bounded and observable. Unbounded silent retries are dangerous.

### Rules for recovery logic

1. **Count attempts** — after N failures, escalate rather than retry
2. **Log each attempt** — operators must see what the system tried
3. **Publish state** — recovery state goes to `/robot/degraded_mode` so it's visible
4. **Respect a timeout** — if not recovered within T seconds, stop and alert operator
5. **Preserve evidence** — trigger a bag record at the moment of fault

```cpp
// Recovery attempt with bounded retry count
int recovery_attempts = 0;
constexpr int max_recovery_attempts = 3;

// In fault handling:
if (recovery_attempts < max_recovery_attempts) {
    RCLCPP_WARN(node->get_logger(), "Recovery attempt %d of %d",
                ++recovery_attempts, max_recovery_attempts);
    // trigger recovery action
} else {
    RCLCPP_ERROR(node->get_logger(),
                 "Recovery exhausted after %d attempts — escalating to STOP",
                 max_recovery_attempts);
    // publish STOP mode, alert operator
}
```
