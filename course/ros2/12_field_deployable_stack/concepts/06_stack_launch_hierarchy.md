# 5. Stack Launch Hierarchy

A production stack has three launch levels:

```
field.launch.py
    ├── drivers.launch.py      (hardware or bridge)
    ├── localization.launch.py
    ├── planning.launch.py
    ├── control.launch.py
    └── monitoring.launch.py
            ├── deployment_checklist_node
            ├── diagnostic_aggregator
            ├── fault_report_hard
            └── degraded_mode_manager
```

Each layer is independently launchable for subsystem testing. The top-level
`field.launch.py` assembles the full stack:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('my_robot')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'monitoring.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'localization.launch.py'))),
    ])
```
