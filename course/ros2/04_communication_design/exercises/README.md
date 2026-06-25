# Exercises - Communication Design

## Moderate Exercises

### Exercise 1 - Interface Selection

For each interaction, choose topic, service, or action: set controller mode, stream battery state, navigate to charging dock, clear local costmap, and publish current mission phase.

### Exercise 2 - QoS Profile Table

Create a QoS table for IMU, LiDAR scan, robot status, velocity command, and map metadata.

### Exercise 3 - Namespace Remapping

Run two instances of the same node under different namespaces and verify their topics do not collide.

## Hard Exercises

### Hard Exercise 1 - Subsystem API Design

Design the ROS2 API for a `mission_manager` subsystem: topics, services, actions, message fields, QoS choices, and failure behavior.

### Hard Exercise 2 - Multi-Robot Naming Review

Given a flat topic layout, rewrite it for two robots using namespaces and remaps. Identify which topics should remain global.

## Starter and Reference Package

- Starter package: `exercises/communication_design_lab/`
- Reference package: `../answers/communication_design_lab/`
- Moderate target: `qos_status_publisher`
- Hard target: `command_api_hard`

## How to Run

See [Running ROS2 Exercises](../../RUNNING_EXERCISES.md) for build commands, package names, executable names, and sample topic inputs.
