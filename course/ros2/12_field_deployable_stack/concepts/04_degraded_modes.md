# 3. Degraded Modes

A field robot must define a safe policy for every fault combination. Not every
fault requires full shutdown — but every fault needs a documented response.

### Mode table

| Fault condition | Safe mode | Rationale |
|---|---|---|
| `control:FAILED` OR `safety:FAILED` | `STOP` | Cannot actuate safely |
| `localization:FAILED` | `LOCALIZE_ONLY` | Can perceive but cannot navigate |
| `sensor:DEGRADED` OR `planning:DEGRADED` | `SLOW` | Reduced confidence → reduced speed |
| All OK | `NORMAL` | Full autonomy |

### Degraded mode selector

```cpp
std::string select_mode(const std::string& health) {
    if (health.find("control:FAILED")   != std::string::npos ||
        health.find("safety:FAILED")    != std::string::npos) {
        return "STOP";
    }
    if (health.find("localization:FAILED") != std::string::npos) {
        return "LOCALIZE_ONLY";
    }
    if (health.find("sensor:DEGRADED")   != std::string::npos ||
        health.find("planning:DEGRADED") != std::string::npos) {
        return "SLOW";
    }
    return "NORMAL";
}
```

```cpp
// Subscribe to aggregated health, publish selected mode every 500 ms
auto sub = node->create_subscription<std_msgs::msg::String>(
    "subsystem/health", 10,
    [health](const std_msgs::msg::String::SharedPtr msg) { *health = msg->data; });

auto timer = node->create_wall_timer(500ms, [output, health] {
    std_msgs::msg::String msg;
    msg.data = select_mode(*health);
    output->publish(msg);
});
```

### Data flow

```
/subsystem/status (multiple publishers)
        │
        ▼
diagnostic_aggregator
        │
        ▼ /subsystem/health "robot_health=DEGRADED sensor:DEGRADED:..."
        │
        ▼
degraded_mode_manager
        │
        ▼ /robot/degraded_mode "SLOW"
        │
        ▼
navigation stack (reads mode, adjusts max velocity / replanning rate)
```
