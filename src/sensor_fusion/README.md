# Sensor Fusion
Sensor fusion in the context of robot localization. This package takes in continuous data from IMU and wheel odometry in arbitrary number, to get a better estimate of position state of the robot. The fusion can be carried out in both two dimensional movement and three dimensional movement. The appropriate configuration can be enabled in the parameter file attached below.  

### Config parameter file
- `sensor_fusion.yaml` - configuration file to setup sensor fusion settings. 
### Subscribed Topics
- `/automaton_velocity_controller/odom (nav_msgs/Odometry)`
- `/imu/data (sensor_msgs/Imu)`
### Published Topics
- `odometry/filtered (nav_msgs/Odometry)`
### Dependencies - trajectory plotter
1. <b>Subscriptions</b> :
    - `/robot-namespace/odom ( nav_msgs/Odometry )` - odometry from the wheel encoder with measurement noise. 
    - `/ground_truth/state ( nav_msgs/Odometry )` - ground truth data from gazebo.
    - `/odometry/filtered ( nav_msgs/Odometry )` - fused and filter out data from the sensor fusion package. 
2. <b>Published</b> :
    - `/odom_path ( nav_msgs/Path )` - odometry data stringed together as path history.
    - `/ground_truth_odom ( nav_msgs/Path )` - ground truth data stringed together as path history.
    - `/ekf_path ( nav_msgs/Path )` - ekf filtered data stringed together as path history.

### Preview of sensor fusion working with the custom robot automaton.

