# ROS2 Module 02 - Package Architecture

## Goal

Design ROS2 workspaces and packages so the robot software can grow without turning into a tightly coupled monolith.

## Package Boundaries

A package should have a clear reason to exist. Common production package categories:

- `robot_interfaces`: custom messages, services, actions
- `robot_bringup`: launch files and top-level orchestration
- `robot_description`: URDF/Xacro, meshes, RViz configs
- `robot_monitoring`: health aggregation and diagnostics
- `robot_localization_app`: localization nodes and configuration
- `robot_navigation_app`: planning and command orchestration

## Dependency Direction

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

## Public vs Private Headers

Use public headers for APIs intended for other packages. Keep implementation details private.

```text
include/my_package/...
src/...
```

## Configuration Ownership

Configuration belongs near the subsystem that understands it, but top-level launch should decide which config file applies to a robot or environment.

## Capstone Direction

Create a multi-package robot application skeleton with clear ownership: interfaces, monitoring node, bringup launch, and config files.

