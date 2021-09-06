# Autonomous-Mobile-Robotics
This repository contains ROS implementation of algorithms thought in the course [Autonomous Mobile Robots](https://www.edx.org/course/autonomous-mobile-robots).
---
### System requirements
1. Ubuntu 20.04 LTS
2. ROS Noetic 

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

### TODO: Perception
1. combine front and rear laser into a single source of laser data
2. create line extraction package to work with lidar
3. use line extraction for localization later 
### TODO: Sensor Fusion
1. Finish Sensor Fusion package (EKF, UKF, and Particle filter)
2. Compare performance between sensor fusion package and robot_pose_ekf package
3. odometry to trajcetory package to compare how odom estimates drift from true state values. 