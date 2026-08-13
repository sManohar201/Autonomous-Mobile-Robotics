# 3. The Bridge Configuration

`ros_gz_bridge` maps Gazebo topic names to ROS2 topic names. Bridge config is
a YAML file — it is integration wiring, not business logic:

```yaml
# bridge.yaml
- topic_name: /world/default/model/robot/.../front_laser/scan
  ros_topic_name: /front_laser/scan
  gz_type_name: gz.msgs.LaserScan
  ros_type_name: sensor_msgs/msg/LaserScan
  direction: GZ_TO_ROS

- topic_name: /model/robot/cmd_vel
  ros_topic_name: /cmd_vel
  gz_type_name: gz.msgs.Twist
  ros_type_name: geometry_msgs/msg/Twist
  direction: ROS_TO_GZ
```

```python
# In launch file
Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{'config_file': bridge_config_path}],
)
```
