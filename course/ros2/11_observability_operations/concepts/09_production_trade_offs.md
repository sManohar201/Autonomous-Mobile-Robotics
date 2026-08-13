# 8. Production Trade-offs

| Decision | Trade-off |
|---|---|
| Single aggregation topic vs multiple | Single `/subsystem/status`: one subscription. Multiple: cleaner per-subsystem routing |
| Text severity vs numeric | Text (`"DEGRADED"`): readable in logs. Numeric: easier to compare in code |
| `RCLCPP_WARN` vs publish to `/faults` | Warn: ephemeral, lost without a log subscriber. Topic: persistent, queryable |
| Continuous bag recording | High storage cost. Use ring-buffer with auto-overwrite or incident-triggered recording |
| Fault with action vs without | With action: actionable for operators. Without: still useful for engineers |
