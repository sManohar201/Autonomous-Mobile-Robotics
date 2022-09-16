# Autonomous-Mobile-Robotics
This repository contains from the scratch implementation of robot autonomy algorithms in ROS2 foxy (I have abandonded my previous work with ROS1(ros-noetic branch)).  

--
### System requirements
1. Ubuntu 20.04 LTS
2. ROS2 Foxy
---
### Installation steps

- `git clone -b ros2-foxy git@github.com:sManohar201/Autonomous-Mobile-Robotics.git`
- `cd Autonomous-Mobile-Robotics`
- `colcon build --symlink-install`
- `source install/setup.bash`

---
### List of topics covered in the course
1. Perception
2. Sensor Fusion
3. Localization
4. Simultaneous Localization and Mapping
5. Path Planning
---
### TODO: Perception
- [ ] feature extraction algorithm, line, blobs, and corners for now.
- [ ] data association for slam and localization.
- [ ] Observation model

### TODO: Sensor Fusion
- [ ] base template for sensor fusion
- [ ] Implement EKF
- [ ] Implement UKF
- [ ] Compare performance between sensor fusion package and robot_pose_ekf package
- [ ] odometry to trajcetory package to compare how odom estimates drift from true state values. 

### TODO: Localization
- [ ] Iterative closest point (should I move this to perception)
- [ ] Kalman filters with iterative solution.
- [ ] Monte Carlo Localization.

### TODO: SLAM
- [ ] Occupancy grid mapping (half done improve bresenham rasterisation algorithm)
- [ ] EKF slam with unknown correspondencies.

### TODO: Path Planning


