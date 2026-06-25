# Exercises - Observability & Operations

## Moderate Exercises

### Exercise 1 - Diagnostic Aggregator

Aggregate several subsystem statuses into one robot health summary.

### Exercise 2 - Fault Report Format

Design a structured fault report with timestamp, subsystem, severity, reason, and suggested action.

### Exercise 3 - Bagging Policy

Define which topics should be recorded during normal operation and during failure investigation.

## Hard Exercises

### Hard Exercise 1 - Operator Health View

Build an operator-facing health summary that distinguishes OK, degraded, and failed subsystems.

### Hard Exercise 2 - Failure Runbook

Write a runbook for stale localization, missing LiDAR, bad IMU frame, and command timeout.

## Starter and Reference Package

- Starter package: `exercises/observability_operations_lab/`
- Reference package: `../answers/observability_operations_lab/`
- Moderate target: `diagnostic_aggregator`
- Hard target: `fault_report_hard`

## How to Run

See [Running ROS2 Exercises](../../RUNNING_EXERCISES.md) for build commands, package names, executable names, and sample topic inputs.
