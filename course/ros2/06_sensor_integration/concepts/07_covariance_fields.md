# 6. Covariance Fields

Many sensor messages carry a covariance matrix expressing measurement uncertainty.
State estimation filters (EKF, UKF) need these values.

### IMU covariance

```cpp
sensor_msgs::msg::Imu msg;
// 3x3 covariance for orientation (row-major)
msg.orientation_covariance[0] = 0.01;   // var(roll)
msg.orientation_covariance[4] = 0.01;   // var(pitch)
msg.orientation_covariance[8] = 0.02;   // var(yaw) — heading noisier
// -1 means "this field is not provided"
msg.linear_acceleration_covariance[0] = -1.0;
```

### Odometry covariance

```cpp
nav_msgs::msg::Odometry odom;
// 6x6 pose covariance [x,y,z,roll,pitch,yaw] (row-major, 36 elements)
odom.pose.covariance[0]  = 0.001;   // var(x)
odom.pose.covariance[7]  = 0.001;   // var(y)
odom.pose.covariance[35] = 0.01;    // var(yaw)
// Twist covariance similarly
odom.twist.covariance[0]  = 0.001;  // var(vx)
odom.twist.covariance[35] = 0.005;  // var(vyaw)
```

> **Interview question:** "A subscriber receives an IMU message with
> `angular_velocity_covariance[0] == -1`. What does that mean?"  
> **Answer:** The field is not provided. Consumers should fall back to a default
> noise model or ignore that component.
