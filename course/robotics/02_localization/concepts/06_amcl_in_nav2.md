# 5. AMCL in Nav2

Nav2's `nav2_amcl` is a particle filter localiser for 2D LiDAR maps.

### Key parameters

```yaml
amcl:
  ros__parameters:
    min_particles: 500
    max_particles: 2000
    pf_err: 0.05           # KLD-sampling error bound
    pf_z: 0.99             # KLD-sampling confidence
    alpha1: 0.2            # motion noise: rotation from rotation
    alpha2: 0.2            # motion noise: rotation from translation
    alpha3: 0.2            # motion noise: translation from translation
    alpha4: 0.2            # motion noise: translation from rotation
    laser_model_type: likelihood_field
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_z_hit: 0.95
    laser_z_rand: 0.05
    set_initial_pose: true
    initial_pose:
      x: 0.0; y: 0.0; yaw: 0.0
```

### Topics

| Topic | Type | Role |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Input LiDAR |
| `/map` | `nav_msgs/OccupancyGrid` | Input map |
| `/odom` | `nav_msgs/Odometry` | Input odometry |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Output best estimate |
| `/particlecloud` | `nav2_msgs/ParticleCloud` | Output all particles |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Input: set initial pose |

### Re-localization

When the robot is kidnapped or loses track:
```bash
# Publish initial pose estimate via CLI
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{pose: {pose: {position: {x: 1.0, y: 0.5}}, covariance: [0.25, 0,0,0,0,0, ...]}}' --once

# Or from RViz: click "2D Pose Estimate" on the map
```
