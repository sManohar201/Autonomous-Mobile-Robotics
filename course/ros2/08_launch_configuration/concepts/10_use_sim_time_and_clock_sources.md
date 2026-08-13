# 9. `use_sim_time` and Clock Sources

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
