# Dependency Direction

Interfaces should sit at the bottom. Application packages depend on interfaces, not the reverse.

Good:

```text
robot_monitoring -> robot_interfaces
robot_navigation_app -> robot_interfaces
robot_bringup -> robot_monitoring, robot_navigation_app
```

Bad:

```text
robot_interfaces -> robot_monitoring
```

Interface packages must remain stable and lightweight.
