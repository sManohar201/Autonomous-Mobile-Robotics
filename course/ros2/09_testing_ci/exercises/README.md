# Exercises - Testing & CI

## Moderate Exercises

### Exercise 1 - Unit Test Freshness Logic

Write unit tests for stale-topic detection without ROS.

### Exercise 2 - Node-Level Test

Start a monitor node and a synthetic publisher, then verify status output.

### Exercise 3 - Launch Test

Write a launch test that verifies required nodes start.

## Hard Exercises

### Hard Exercise 1 - Bag Replay Regression

Record a short bag and use it as a regression fixture for sensor-health behavior.

### Hard Exercise 2 - CI Test Matrix

Design a CI matrix with build, unit test, launch test, lint, and optional simulation jobs.

## Starter and Reference Package

- Starter package: `exercises/testing_ci_lab/`
- Reference package: `../answers/testing_ci_lab/`
- Moderate target: `freshness_logic_node`
- Hard target: `test_report_hard`

## How to Run

See [Running ROS2 Exercises](../../RUNNING_EXERCISES.md) for build commands, package names, executable names, and sample topic inputs.
