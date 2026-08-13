# 4. Configuration Layers

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
