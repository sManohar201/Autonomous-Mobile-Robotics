# Exercises - Production Nodes

## Moderate Exercises

### Exercise 1 - Parameter Validation Node

Create a node with parameters `input_topic`, `output_topic`, `expected_rate_hz`, and `stale_timeout_ms`. Reject invalid values and log clear messages.

### Exercise 2 - Lifecycle State Logging

Create a lifecycle node and log configure, activate, deactivate, cleanup, and shutdown transitions.

### Exercise 3 - Diagnostic Status Publisher

Publish a simple diagnostic/status message with OK, DEGRADED, FAILED, and summary text.

## Hard Exercises

### Hard Exercise 1 - Lifecycle Sensor Processor

Build a lifecycle-managed processor that declares and validates parameters, only subscribes after activation, stops publishing after deactivation, and reports stale input as degraded.

### Hard Exercise 2 - Failure Mode Table

Write a failure-mode table for invalid config, missing input, stale input, publish failure, and shutdown during active processing. For each, define detection, log level, diagnostic level, and recovery behavior.

## Starter and Reference Package

- Starter package: `exercises/production_nodes_lab/`
- Reference package: `../answers/production_nodes_lab/`
- Moderate target: `parameter_validation_node`
- Hard target: `lifecycle_processor_hard`

## How to Run

See [Running ROS2 Exercises](../../RUNNING_EXERCISES.md) for build commands, package names, executable names, and sample topic inputs.
