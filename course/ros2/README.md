# ROS2 for Field-Deployable Robotics Systems

This course section teaches ROS2 as a systems architecture tool for production robotics. The focus is not only writing nodes, but designing deployable robot software: package boundaries, interfaces, lifecycle, observability, configuration, testing, simulation integration, and failure handling.

Gazebo is introduced gradually. Early chapters use deterministic ROS2 examples so the architecture is clear. Simulation becomes the integration environment after the core node, interface, lifecycle, and testing patterns are established.

## Running Exercises

See [Running ROS2 Exercises](RUNNING_EXERCISES.md) for build commands, `ros2 run` commands, package names, executable names, and sample topic inputs.

## Modules

| # | Module | Focus | Capstone |
|---|--------|-------|----------|
| 01 | [ROS2 Foundations](01_ros2_foundations/concepts.md) | Graph, nodes, topics, services, actions, parameters, launch | `robot_monitor` prototype |
| 02 | [Package Architecture](02_package_architecture/concepts.md) | Workspace layout, package boundaries, interface ownership | multi-package robot app skeleton |
| 03 | [Production Nodes](03_production_nodes/concepts.md) | Lifecycle, validation, logging, diagnostics, failure paths | lifecycle-managed sensor processor |
| 04 | [Communication Design](04_communication_design/concepts.md) | Topic/service/action tradeoffs, QoS, namespaces, remapping | command and telemetry API |
| 05 | [Executors & Concurrency](05_executors_concurrency/concepts.md) | Callback groups, executors, timers, threading, safe shared state | concurrent sensor pipeline |
| 06 | [Sensor Integration](06_sensor_integration/concepts.md) | LiDAR, IMU, odom, TF2, timestamps, frame semantics | sensor health and synchronization pipeline |
| 07 | [State Estimation Architecture](07_state_estimation_architecture/concepts.md) | Filter interfaces, measurement adapters, covariance, reset/reinit | localization subsystem shell |
| 08 | [Launch & Configuration](08_launch_configuration/concepts.md) | Launch orchestration, YAML, variants, environments | field/development launch stack |
| 09 | [Testing & CI](09_testing_ci/concepts.md) | Unit tests, launch tests, bag replay, simulation tests | regression test suite |
| 10 | [Gazebo Integration](10_gazebo_integration/concepts.md) | Bridges, sim time, robot model, control/sensor topics | sim-integrated robot monitor |
| 11 | [Observability & Operations](11_observability_operations/concepts.md) | Diagnostics, logs, bags, metrics, fault reports | operator-facing health system |
| 12 | [Field Deployable Stack](12_field_deployable_stack/concepts.md) | Deployment architecture, degraded modes, recovery, runbooks | production-grade robot stack skeleton |

## Chapter Composition

Each chapter contains:

- `concepts.md` for architecture principles and implementation patterns
- `quiz.md` for review and design reasoning
- `exercises/README.md` with moderate and hard exercises
- `answers/` with reference implementations for the exercise packages

Moderate exercises reinforce the core ROS2 concept. Hard exercises require production-oriented design decisions, failure handling, and integration tradeoffs.
