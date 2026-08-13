# 8. Gazebo Sensor Bridge

In simulation, sensor data originates from Gazebo plugins and is bridged to ROS2
topics by `ros_gz_bridge`. The bridge runs as a separate process and maps
Gazebo topic names to ROS2 topic names.

```
Gazebo plugin (gpu_lidar)
    │ /world/default/model/robot/link/laser_link/sensor/front_laser/scan [gz::msgs::LaserScan]
    ▼
ros_gz_bridge
    │ /front_laser/scan [sensor_msgs/LaserScan]
    ▼
Your ROS2 node
```

The bridge handles timestamp conversion: Gazebo wall time → ROS2 `header.stamp`.
With `use_sim_time: true`, timestamps come from the Gazebo `/clock` topic.

### Verifying sensor topics in simulation

```bash
# List all active topics
ros2 topic list

# Check type and rate
ros2 topic info /front_laser/scan --verbose
ros2 topic hz /imu/data

# Inspect a message
ros2 topic echo /odom --once
```

### Common bridge issues

| Symptom | Cause | Fix |
|---|---|---|
| No messages on `/imu/data` | Bridge not running or topic name mismatch | Check `ros2 topic list` vs Gazebo topic names |
| Timestamps at time 0 | `use_sim_time` mismatch | Ensure all nodes have matching clock source |
| Wrong frame_id | URDF frame != bridge remapping | Match URDF sensor frame names to plugin config |
