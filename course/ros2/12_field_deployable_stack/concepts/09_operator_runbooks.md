# 8. Operator Runbooks

For each fault mode, document the runbook in a human-readable format:

**STOP mode:**
1. Robot has halted. Do not attempt to drive.
2. Check `/operator/fault_report` for reason.
3. If `control:FAILED`: inspect CAN bus and motor driver logs.
4. Download incident bag for engineering review.
5. Contact on-call engineer before restarting.

**LOCALIZE_ONLY mode:**
1. Robot can perceive but not navigate autonomously.
2. Check `/localization/status` for `accepted` count and `update_age_s`.
3. If `update_age_s > 5`: check odometry topic with `ros2 topic hz /odom`.
4. Reset localization: `ros2 service call /localization/reset std_srvs/srv/Trigger`
5. If reset fails to recover within 60 s, escalate to STOP.
