# 1. Observability Signals

Production robot systems need five types of observability:

| Signal | Form | Who reads it | Tool |
|---|---|---|---|
| **Diagnostics** | Structured health status per subsystem | Operators, dashboards | `/diagnostic_agg`, custom topics |
| **Logs** | Timestamped text per severity | Engineers debugging | `rcl_logging`, `ros2 topic echo /rosout` |
| **Bags** | Full topic recordings | Post-mortem analysis | `ros2 bag record` |
| **Metrics** | Numerical counters / rates | Monitoring dashboards | Custom publishers |
| **Fault reports** | Structured fault + action | Operators in the field | Custom topic |
