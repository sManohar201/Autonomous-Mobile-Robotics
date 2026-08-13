# 9. Testing Sensor Integration without Hardware

Use `ros2 topic pub` to inject synthetic sensor data:

```bash
# Publish a synthetic IMU message
ros2 topic pub /imu/data sensor_msgs/msg/Imu \
  '{header: {frame_id: imu_link}, angular_velocity: {z: 0.1}}' \
  --rate 100

# Publish with wrong frame to test frame_validator
ros2 topic pub /imu/data sensor_msgs/msg/Imu \
  '{header: {frame_id: wrong_frame}}' \
  --rate 10
```

For automated tests, use `rclcpp::executors::SingleThreadedExecutor` with a
`WallTimer` that publishes test messages and another timer that reads the output.
See Module 09 (Testing & CI) for the full pattern.
