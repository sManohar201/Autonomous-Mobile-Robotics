# 5. Data Flow in This Repository

```
keyboard/nav stack
   │ /cmd_vel (Twist)
   ▼
ros_gz_bridge ──────────────────► Gazebo Harmonic (gz-sim)
                                    DiffDrive plugin → wheel joints, /odom
                                    gpu_lidar (2D) → /front_laser/scan
                                    gpu_lidar (3D) → /top_lidar_3d/points
                                    IMU / GPS / Magnetometer
ros_gz_bridge ◄─── all gz topics ──┘
robot_state_publisher ◄─ /joint_states (ros) ─ joint_state_publisher (ROS timer)
rviz2 ◄─ TF tree, sensor topics
```

Key design decision: `/tf` and `/joint_states` are NOT bridged from Gazebo.
Instead, `joint_state_publisher` (a ROS timer) is the sole source of
`/joint_states` — this prevents non-monotonic timestamps from causing the
`tf2` buffer to clear and RViz to blink.
