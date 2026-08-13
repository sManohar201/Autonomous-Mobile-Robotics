# 3. Frame IDs and TF2

### Why frame IDs matter

A sensor measurement without a frame ID is a number without units — useless to
any consumer that needs to relate it to robot pose.

```
/imu/data.header.frame_id = "imu_link"
               means:
"this linear acceleration vector is expressed in the imu_link frame"
```

A consumer wanting IMU data in `base_link` coordinates must look up the
`imu_link` → `base_link` transform from TF2.

### Validating frame IDs

The simplest check: compare the actual `frame_id` to the expected value.

```cpp
auto expected = node->declare_parameter<std::string>("expected_frame_id", "imu_link");
auto sub = node->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10,
    [pub, expected](const sensor_msgs::msg::Imu::SharedPtr msg) {
        std_msgs::msg::String status;
        status.data = (msg->header.frame_id == expected)
            ? "OK"
            : "FRAME_ERROR expected=" + expected +
              " actual=" + msg->header.frame_id;
        pub->publish(status);
    });
```

Make `expected_frame_id` a parameter so the same node works for any sensor
without recompilation.

### The TF2 tree

```
map
 └── odom
      └── base_link
           ├── imu_link
           ├── laser_link
           ├── lidar_link
           └── gps_link
```

Each edge is a transform published by a TF2 broadcaster. The robot description
(URDF/xacro) defines the static edges. Dynamic edges (odom → base_link) are
published by wheel odometry or a filter.

```cpp
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

auto tf_buffer    = std::make_shared<tf2_ros::Buffer>(node->get_clock());
auto tf_listener  = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

// Look up imu_link pose in base_link at a specific time
auto tf = tf_buffer->lookupTransform("base_link", "imu_link",
                                      tf2::TimePointZero);
```

> **See also:** https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html
