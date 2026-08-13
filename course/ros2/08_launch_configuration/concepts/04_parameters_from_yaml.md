# 3. Parameters from YAML

Inline parameters in a launch file are acceptable for small sets. For anything
larger, use a YAML file:

```yaml
# config/dev.yaml
config_echo_node:
  ros__parameters:
    robot_name: robot
    config_profile: dev
    status_timeout_s: 1.0
```

```python
# In the launch file:
import os
from ament_index_python.packages import get_package_share_directory

config_dir = get_package_share_directory('my_pkg')
config_file = os.path.join(config_dir, 'config', 'dev.yaml')

Node(
    package='my_pkg',
    executable='config_echo_node',
    parameters=[config_file],  # load from YAML
)
```

### YAML structure rules

- Top-level key is the node name (as declared, not the executable)
- Under `ros__parameters:` (double underscore)
- Nested parameters use dot notation in the YAML key: `sensor.rate_hz`

```yaml
my_node:
  ros__parameters:
    robot_name: robot
    sensor:
      rate_hz: 50.0
      frame_id: laser_link
```
