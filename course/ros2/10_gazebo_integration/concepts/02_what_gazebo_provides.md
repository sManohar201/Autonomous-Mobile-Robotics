# 1. What Gazebo Provides

Gazebo Harmonic runs a physics simulation with:
- A robot model (URDF/SDF) — joints, links, inertia, collision geometry
- Sensor plugins (GPU LiDAR, IMU, odometry, GPS)
- Actuator plugins (differential drive, joint controllers)
- A simulation clock published on `/clock`

ROS2 nodes see none of this directly — they see ROS2 topics bridged from Gazebo.

```
Gazebo Harmonic
    │
    │ Gazebo transport (gz::msgs)
    │
ros_gz_bridge
    │
    │ ROS2 DDS
    │
Your node (same code as on hardware)
```
