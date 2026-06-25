# Exercises - Launch & Configuration

## Moderate Exercises

### Exercise 1 - Parameter YAML

Create a YAML file for `robot_monitor` expected rates and timeouts.

### Exercise 2 - Launch Arguments

Write a launch file with `robot_name`, `use_sim_time`, and `config_file` arguments.

### Exercise 3 - Namespaced Launch

Launch the same node under `/robot1` and `/robot2`.

## Hard Exercises

### Hard Exercise 1 - Environment Split

Create separate launch paths for development, simulation, and field deployment.

### Hard Exercise 2 - Lifecycle Bringup

Design a launch flow that configures and activates lifecycle nodes in a safe order.

## Starter and Reference Package

- Starter package: `exercises/launch_configuration_lab/`
- Reference package: `../answers/launch_configuration_lab/`
- Moderate target: `config_echo_node`
- Hard target: `variant_bringup_hard`

## How to Run

See [Running ROS2 Exercises](../../RUNNING_EXERCISES.md) for build commands, package names, executable names, and sample topic inputs.
