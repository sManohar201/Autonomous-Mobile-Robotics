# Exercises - State Estimation Architecture

## Moderate Exercises

### Exercise 1 - Measurement Adapter

Design an adapter from an odometry-like message into an internal `PoseMeasurement`.

### Exercise 2 - Filter Interface

Define a non-ROS `StateEstimator` interface with `predict`, `update`, `reset`, and `state`.

### Exercise 3 - Reset Service Design

Specify request/response fields for a localization reset service.

## Hard Exercises

### Hard Exercise 1 - Localization Subsystem Shell

Build a ROS-facing localization node that owns adapters, delegates math to a filter core, exposes reset, and publishes diagnostics.

### Hard Exercise 2 - Rejection Policy

Design a measurement rejection policy for stale timestamps, wrong frames, high covariance, and outlier position jumps.

## Starter and Reference Package

- Starter package: `exercises/state_estimation_architecture_lab/`
- Reference package: `../answers/state_estimation_architecture_lab/`
- Moderate target: `measurement_adapter`
- Hard target: `localization_shell_hard`

## How to Run

See [Running ROS2 Exercises](../../RUNNING_EXERCISES.md) for build commands, package names, executable names, and sample topic inputs.
