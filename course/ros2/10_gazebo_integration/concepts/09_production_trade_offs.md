# 8. Production Trade-offs

| Decision | Trade-off |
|---|---|
| Bridge YAML vs bridge args | YAML: versionable, auditable. Args: quick for one-off debugging |
| `use_sim_time` for all nodes | All nodes advance together with Gazebo — required for correct timestamps |
| Startup delay vs readiness probe | `TimerAction(5s)` is simple but fragile if Gazebo is slow. Readiness probe is correct but more code |
| Monitoring clock directly vs via node->now() | Direct `/clock` sub works even before the node clock is synchronised |
| Separate sim launch vs `IfCondition` | Separate: clear ownership. IfCondition: one file but more logic |
