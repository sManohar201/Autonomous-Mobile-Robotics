# 7. Operational Workflows

Every fault class should have a documented workflow:

```
Fault: DEGRADED (stale sensor)
  1. Detect: subsystem/status shows STALE_INPUTS
  2. Report: fault_report published with reason + action
  3. Degrade: localization enters lower-confidence mode (wider covariance)
  4. Recover: if sensor comes back within 5 s, auto-resume
  5. Evidence: bag of last 60 s automatically preserved
  6. Stop: if no recovery after 30 s, halt navigation, wait for operator
```

This workflow is a contract — the software must implement each step and the
operator's runbook must match.
