# 8. Architecture Diagram

```
/odom topic
    │ nav_msgs/Odometry
    ▼
[adapt_odometry()]  ←── extract x, y, yaw, cov_trace
    │ PoseMeasurement
    ▼
[StateEstimator::predict(stamp)]
[StateEstimator::update(msg, &reason)]
    │ accepted / rejected
    ├──► RCLCPP_WARN_THROTTLE (on rejection)
    │
    ├──► [Timer 500ms]
    │         └──► /localization/status (diagnostics)
    │
    └──► [Reset service]
              └── /localization/reset → estimator.reset()
```
