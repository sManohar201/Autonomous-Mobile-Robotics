# ROS2 Module 08 — Launch & Configuration

## Goal

Design launch files and parameter configurations that support development,
simulation, and field deployment without changing source code. A robot codebase
running on multiple platforms needs a configuration layer that selects the right
parameters at startup, not at compile time.

> **See also:** https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html
> for the official launch tutorial. This module focuses on the architectural
> decisions that make launch files maintainable at scale.

---

## 1. What Launch Files Are For

A launch file starts a graph of ROS2 nodes with specific parameters, remaps, and
namespace assignments. It is not just a convenience wrapper around `ros2 run` — it
is the **deployment configuration** for your subsystem.

```
launch file
    │
    ├── parameters (from YAML or inline)
    ├── remaps (topic name → canonical name)
    ├── namespace (all topics scoped under /robot_name/)
    ├── node A
    ├── node B
    └── conditional inclusion (sim only, field only)
```

Three things that belong in a launch file, not in node code:
1. Which nodes to start
2. What parameters each node receives at startup
3. Topic remaps for connecting nodes with different naming conventions

---

## 2. Launch File Anatomy (Python)

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

---

## 3. Parameters from YAML

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

---

## 4. Configuration Layers

Production robots need layered configuration:

```
base package defaults (declared in node code)
    │  lowest priority
    ▼
package default YAML  (config/defaults.yaml)
    │
    ▼
robot-specific YAML   (config/robots/pallet_jack.yaml)
    │
    ▼
environment YAML      (config/envs/field.yaml)
    │
    ▼
operator overrides    (--ros-args -p key:=value)
    │  highest priority
```

Load multiple config files — later files override earlier ones:

```python
Node(
    package='my_pkg',
    executable='monitor',
    parameters=[
        os.path.join(config_dir, 'config', 'defaults.yaml'),
        os.path.join(config_dir, 'config', 'robots', robot_name + '.yaml'),
        os.path.join(config_dir, 'config', 'envs', environment + '.yaml'),
    ],
)
```

---

## 5. Configuring Nodes from Parameters

Nodes should read all tunable values from parameters, not from compile-time
constants or environment variables.

```cpp
#include <memory>
#include <sstream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("config_echo_node");

    node->declare_parameter<std::string>("robot_name", "robot");
    node->declare_parameter<std::string>("config_profile", "dev");
    node->declare_parameter<double>("status_timeout_s", 1.0);

    auto pub = node->create_publisher<std_msgs::msg::String>("config/summary", 10);
    auto timer = node->create_wall_timer(std::chrono::seconds(1), [node, pub] {
        std::ostringstream out;
        out << "robot="     << node->get_parameter("robot_name").as_string()
            << " profile="  << node->get_parameter("config_profile").as_string()
            << " timeout_s=" << node->get_parameter("status_timeout_s").as_double()
            << " use_sim_time=" << (node->get_parameter("use_sim_time").as_bool()
                                   ? "true" : "false");
        std_msgs::msg::String msg;
        msg.data = out.str();
        pub->publish(msg);
    });

    (void)timer;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

`use_sim_time` is a built-in parameter — always inject it via
`parameters=[{'use_sim_time': True}]` in the launch file, not in node code.

---

## 6. Environment Variants

Some nodes need to behave differently in `dev`, `sim`, and `field` environments.
The node should never hardcode "if running on hardware, do X" — instead, read
an `environment` parameter and dispatch:

```cpp
bool is_supported_environment(const std::string& env) {
    constexpr std::array<const char*, 3> allowed{"dev", "sim", "field"};
    for (const auto* e : allowed) { if (env == e) return true; }
    return false;
}

std::string lifecycle_policy_for(const std::string& env) {
    if (env == "field") {
        return "configure:drivers,monitoring,localization; activate:all";
    }
    if (env == "sim") {
        return "configure:gazebo_bridge,monitoring,localization; activate:all";
    }
    return "configure:monitoring; activate:monitoring";  // dev
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("variant_bringup_selector");
    node->declare_parameter<std::string>("environment", "dev");
    node->declare_parameter<std::string>("robot_name", "robot");

    const auto env = node->get_parameter("environment").as_string();
    if (!is_supported_environment(env)) {
        RCLCPP_FATAL(node->get_logger(), "Unsupported environment: %s", env.c_str());
        rclcpp::shutdown();
        return 2;
    }
    // ... publish selected profile
}
```

Exit code 2 signals "misconfiguration" — distinct from 1 (runtime error)
and 0 (success). CI pipelines can detect this.

---

## 7. Conditional Node Inclusion

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

---

## 8. Namespaces and Remaps

### Namespaces

All topics from a node inside a namespace are prefixed:

```python
Node(
    package='my_pkg',
    executable='monitor',
    namespace='pallet_jack',
    # topics: /pallet_jack/config/summary, /pallet_jack/sensor/health
)
```

Use namespaces when multiple robots share a network — without them, topic names
collide.

### Remaps

Remaps connect a node that publishes `/odom` to a consumer that subscribes to
`/robot/odom` without changing either node:

```python
Node(
    package='my_pkg',
    executable='monitor',
    remappings=[
        ('/odom', '/robot/odom'),
        ('/cmd_vel', '/robot/cmd_vel'),
    ],
)
```

> **Production trade-off:** Over-remapping creates a hidden dependency graph that
> is hard to trace. Prefer canonical topic names in the node and namespace scoping
> in the launch file. Only remap when integrating third-party nodes with fixed
> topic names.

---

## 9. `use_sim_time` and Clock Sources

When running in Gazebo, all nodes must use simulation time:

```python
Node(
    package='my_pkg',
    executable='monitor',
    parameters=[{'use_sim_time': True}],
)
```

Without this, `node->now()` returns wall time while sensor timestamps come from
the Gazebo clock — staleness checks will incorrectly flag all sensors as stale.

Verify the clock source at runtime:
```bash
ros2 param get /monitor use_sim_time   # should print: True
ros2 topic hz /clock                   # should print ~1000 Hz in Gazebo
```

---

## 10. Development vs Field Launch Stacks

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

---

## 11. Production Trade-offs

| Decision | Trade-off |
|---|---|
| Inline params vs YAML files | Inline: visible in launch file. YAML: versionable, shareable, robot-specific |
| Single launch file vs variants | Single with args: fewer files. Variants: clearer intent, easier to review per environment |
| Hardcoded topic names vs remaps | Hardcoded: simpler. Remaps: flexible integration, harder to trace |
| `IfCondition` vs separate launch files | `IfCondition`: one file for all cases. Separate: cleaner, no conditional logic |
| Namespace per robot vs no namespace | Namespace: required for multi-robot. No namespace: simpler single-robot setups |

---

## 12. Checklist

- [ ] Every tunable value is a declared parameter with a sensible default
- [ ] Config YAML files are installed by `CMakeLists.txt` via `install(DIRECTORY config DESTINATION share/${PROJECT_NAME})`
- [ ] `use_sim_time` is set in the launch file, not in node code
- [ ] `environment` parameter is validated in node code — exit 2 on unknown value
- [ ] Development and field launch files are separate — no `if debug:` logic in launch Python
- [ ] Namespace used when multiple instances of the node can run simultaneously
