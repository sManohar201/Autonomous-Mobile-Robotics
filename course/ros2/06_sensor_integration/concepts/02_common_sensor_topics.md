# 1. Common Sensor Topics

### Message types and frames

| Topic | Message type | Typical frame | Typical rate |
|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | `imu_link` | 100 Hz |
| `/odom` | `nav_msgs/Odometry` | `odom` → `base_link` | 50 Hz |
| `/front_laser/scan` | `sensor_msgs/LaserScan` | `laser_link` | 10–50 Hz |
| `/top_lidar_3d/points` | `sensor_msgs/PointCloud2` | `lidar_link` | 10 Hz |
| `/gps/fix` | `sensor_msgs/NavSatFix` | `gps_link` | 10 Hz |
| `/magnetometer` | `sensor_msgs/MagneticField` | `imu_link` | 50 Hz |

The topic name and message type are the interface contract. The frame ID and
timestamp inside the message are the data contract. Both must be correct.

### Include paths

```cpp
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
```

In `package.xml`:
```xml
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
<depend>geometry_msgs</depend>
```
