# Exercises - Executors & Concurrency

## Moderate Exercises

### Exercise 1 - Timer and Subscriber Node

Create a node with one subscriber, one timer, latest-message storage, and mutex-protected access.

### Exercise 2 - Callback Group Review

For a node with subscriptions, services, and timers, assign callback groups and justify each choice.

### Exercise 3 - Snapshot Processing

Implement a processing timer that copies latest data under lock, then releases the lock before computing.

## Hard Exercises

### Hard Exercise 1 - Multi-Threaded Executor Safety

Run a node under a multi-threaded executor and identify which callbacks may overlap. Add synchronization or callback group constraints.

### Hard Exercise 2 - Concurrent Sensor Pipeline

Build a sensor pipeline node with LiDAR cache, IMU cache, odom cache, processing timer, minimal lock scope, and diagnostics for missing inputs.

## Starter and Reference Package

- Starter package: `exercises/executors_concurrency_lab/`
- Reference package: `../answers/executors_concurrency_lab/`
- Moderate target: `snapshot_processor`
- Hard target: `concurrent_pipeline_hard`

## How to Run

See [Running ROS2 Exercises](../../RUNNING_EXERCISES.md) for build commands, package names, executable names, and sample topic inputs.
