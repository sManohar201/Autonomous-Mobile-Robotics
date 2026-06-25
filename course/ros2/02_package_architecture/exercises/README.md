# Exercises - Package Architecture

## Moderate Exercises

### Exercise 1 - Package Boundary Sketch

Design a package layout for a robot with a LiDAR driver, IMU driver, health monitor, localization, and top-level launch.

### Exercise 2 - Interface Package

Create a `robot_course_interfaces` package containing a `RobotStatus.msg`.

Suggested fields:

- `builtin_interfaces/Time stamp`
- `uint8 level`
- `string summary`
- `string[] stale_topics`

### Exercise 3 - Bringup Package

Create a bringup package that launches one monitoring node with parameters loaded from YAML.

## Hard Exercises

### Hard Exercise 1 - Dependency Audit

Given a proposed dependency graph, identify invalid dependency directions and rewrite the graph.

### Hard Exercise 2 - Production Workspace Skeleton

Create a workspace layout with `robot_course_interfaces`, `robot_course_monitoring`, `robot_course_bringup`, `config/`, and a launch file that wires monitoring parameters.

Explain why each package exists and what it must not depend on.

## Starter and Reference Package

- Starter package: `exercises/package_architecture_lab/`
- Reference package: `../answers/package_architecture_lab/`
- Moderate target: `package_boundary_review`
- Hard target: `dependency_audit_hard`

## How to Run

See [Running ROS2 Exercises](../../RUNNING_EXERCISES.md) for build commands, package names, executable names, and sample topic inputs.
