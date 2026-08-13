# 8. Namespaces and Remaps

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
