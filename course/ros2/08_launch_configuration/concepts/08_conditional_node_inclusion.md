# 7. Conditional Node Inclusion

Some nodes only make sense in simulation (Gazebo bridge) or on hardware (CAN
bus driver). Use `IfCondition` to conditionally include them:

```python
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression

use_sim_arg = DeclareLaunchArgument('use_sim', default_value='false')

gazebo_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    condition=IfCondition(LaunchConfiguration('use_sim')),
    # only started when use_sim:=true
)

hardware_driver = Node(
    package='can_driver',
    executable='driver',
    condition=UnlessCondition(LaunchConfiguration('use_sim')),
    # only started when use_sim:=false
)
```
