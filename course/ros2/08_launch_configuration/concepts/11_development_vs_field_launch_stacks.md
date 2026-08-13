# 10. Development vs Field Launch Stacks

A mature codebase has two launch files for the same subsystem:

**`dev.launch.py`** — for local development:
- `use_sim_time: false`
- Verbose logging (`--ros-args --log-level DEBUG`)
- No Gazebo (test with `ros2 topic pub` manually)
- All nodes in the same process for easy attach-with-debugger

**`field.launch.py`** — for hardware:
- `use_sim_time: false`
- `config_profile: field`
- Loads hardware-specific YAML
- Uses lifecycle managed nodes for controlled startup

```python
# field.launch.py
def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    config = os.path.join(get_package_share_directory('my_pkg'),
                          'config', 'field.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='pallet_jack'),
        Node(
            package='my_pkg',
            executable='monitor',
            parameters=[config, {'robot_name': robot_name}],
        ),
    ])
```
