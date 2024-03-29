# Autonomous-Mobile-Robotics
This repository contains from the scratch ROS implementation of robot autonomy algorithms.  
---
### System requirements
1. Ubuntu 20.04 LTS
2. ROS Noetic 
---
### Installation steps

- `sudo apt-get install ros-noetic-hector*`
- `git clone git@github.com:sManohar201/Autonomous-Mobile-Robotics.git`
- `cd Autonomous-Mobile-Robotics`
- `catkin_make` or `catkin build`
- `source devel/setup.bash`

---
### List of topics covered in the course
1. Perception
2. Sensor Fusion
3. Localization
4. Simultaneous Localization and Mapping
5. Path Planning
---
### TODO: Perception
- [x] combine front and rear laser into a single source of laser data.
- [ ] feature extraction algorithm, line, blobs, and corners for now.
- [ ] data association for slam and localization.
- [ ] Observation model

### TODO: Sensor Fusion
- [x] base template for sensor fusion
- [x] Implement EKF
- [ ] Implement UKF
- [x] Compare performance between sensor fusion package and robot_pose_ekf package
- [x] odometry to trajcetory package to compare how odom estimates drift from true state values. 

### TODO: Localization
- [ ] Iterative closest point (should I move this to perception)
- [ ] Kalman filters with iterative solution.
- [ ] Monte Carlo Localization.

### TODO: SLAM
- [X] Occupancy grid mapping (half done improve bresenham rasterisation algorithm)
- [ ] EKF slam with unknown correspondencies.

### TODO: Path Planning


