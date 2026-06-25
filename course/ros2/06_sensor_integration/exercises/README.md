# Exercises - Sensor Integration

## Moderate Exercises

### Exercise 1 - Sensor Topic Inventory

Run the robot simulation or synthetic publishers and document topic name, message type, frame ID, expected rate, and QoS profile.

### Exercise 2 - Stale Topic Detector

Implement stale detection for one topic using node time.

Parameters:

- topic name
- stale timeout
- expected frame ID

### Exercise 3 - Frame Validator

Subscribe to a sensor topic and warn when frame ID is empty or unexpected.

## Hard Exercises

### Hard Exercise 1 - Sensor Health Aggregator

Build a node that tracks IMU, odom, and scan: message count, measured rate, last timestamp, stale status, and frame validity. Publish a health summary.

### Hard Exercise 2 - Synthetic First, Gazebo Second

Run the same health aggregator against synthetic test publishers and Gazebo-provided topics. Document what changed: QoS, timing, frame IDs, startup ordering, and sim time.

## Starter and Reference Package

- Starter package: `exercises/sensor_integration_lab/`
- Reference package: `../answers/sensor_integration_lab/`
- Moderate target: `frame_validator`
- Hard target: `sensor_health_hard`

## How to Run

See [Running ROS2 Exercises](../../RUNNING_EXERCISES.md) for build commands, package names, executable names, and sample topic inputs.
