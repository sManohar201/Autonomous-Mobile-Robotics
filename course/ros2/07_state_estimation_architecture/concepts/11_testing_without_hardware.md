# 10. Testing Without Hardware

Use `ros2 topic pub` to inject synthetic odometry:

```bash
# Inject a valid odom message
ros2 topic pub /odom nav_msgs/msg/Odometry \
  '{header: {frame_id: odom}, pose: {pose: {position: {x: 1.0, y: 2.0}}}}' \
  --once

# Inject wrong frame to test rejection
ros2 topic pub /odom nav_msgs/msg/Odometry \
  '{header: {frame_id: wrong_frame}}' \
  --once

# Check diagnostics
ros2 topic echo /localization/status --once

# Reset
ros2 service call /localization/reset std_srvs/srv/Trigger
```
