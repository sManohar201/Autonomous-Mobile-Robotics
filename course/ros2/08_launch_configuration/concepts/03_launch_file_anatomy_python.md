# 2. Launch File Anatomy (Python)

ROS2 launch files are Python scripts returning a `LaunchDescription`.

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='robot',
        description='Name of the robot — used as namespace')

    config_profile_arg = DeclareLaunchArgument(
        'config_profile', default_value='dev',
        description='Configuration profile: dev, sim, or field')

    monitor_node = Node(
        package='my_pkg',
        executable='monitor',
        name='monitor',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[{
            'robot_name':    LaunchConfiguration('robot_name'),
            'config_profile': LaunchConfiguration('config_profile'),
            'status_timeout_s': 1.0,
        }],
    )

    return LaunchDescription([
        robot_name_arg,
        config_profile_arg,
        monitor_node,
    ])
```

### Running with arguments

```bash
ros2 launch my_pkg bringup.launch.py robot_name:=pallet_jack config_profile:=field
```
