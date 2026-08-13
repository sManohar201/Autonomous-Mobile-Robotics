# 11. Summary — What a Field-Deployable Stack Looks Like

```
Startup:  driver launch → bridge → robot_state_publisher → navigation stack
Config:   robot YAML + env YAML + operator overrides
Monitor:  each subsystem → /subsystem/status → aggregator → /operator/health
Degrade:  aggregator → /subsystem/health → degraded_mode_manager → /robot/degraded_mode
Recover:  bounded retries + RCLCPP_WARN per attempt + bag on fault
Evidence: continuous bag (ring buffer) + incident bag on STOP/FAILED
Runbooks: documented per fault mode, match what the code actually does
```

A stack is field-deployable when operators can read its health, understand
its faults, act on its runbooks, and engineers can reproduce any incident
from a bag.
