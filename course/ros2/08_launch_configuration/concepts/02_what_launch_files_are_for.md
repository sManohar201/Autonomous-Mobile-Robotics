# 1. What Launch Files Are For

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
