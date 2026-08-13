# 7. Production SLAM Systems

| System | Backend | Front-end | Use case |
|---|---|---|---|
| Cartographer (Google) | Graph SLAM (g2o) | 2D/3D LiDAR | Warehouse, outdoor |
| RTAB-Map | Graph SLAM | LiDAR + camera | Indoor, loop closure |
| GMapping | Particle filter | 2D LiDAR | Simple indoor |
| KISS-ICP | ICP + point cloud | 3D LiDAR | Fast odometry |
| LIO-SAM | Factor graph (GTSAM) | LiDAR + IMU | Outdoor 3D |

### Cartographer in ROS2

```bash
ros2 launch cartographer_ros cartographer.launch.py \
  use_sim_time:=true \
  configuration_directory:=/path/to/config \
  configuration_basename:=my_robot.lua
```

Key configuration parameters:
```lua
-- my_robot.lua
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
POSE_GRAPH.optimize_every_n_nodes = 90
```
