# Exercises - Gazebo Integration

## Moderate Exercises

### Exercise 1 - Topic Comparison

Compare synthetic sensor topics with Gazebo sensor topics: names, types, QoS, frames, and rates.

### Exercise 2 - Sim Time Check

Run a node with and without `use_sim_time` and document timer behavior.

### Exercise 3 - Bridge Inventory

Document every Gazebo-to-ROS bridge used by the robot simulation.

## Hard Exercises

### Hard Exercise 1 - Sim-Integrated Monitor

Run `robot_monitor` against Gazebo topics and handle startup delays without false failures.

### Hard Exercise 2 - Simulation Isolation Review

Identify which launch/config files are simulation-specific and which code should remain hardware-compatible.

## Starter and Reference Package

- Starter package: `exercises/gazebo_integration_lab/`
- Reference package: `../answers/gazebo_integration_lab/`
- Moderate target: `sim_time_check`
- Hard target: `gazebo_monitor_hard`

## How to Run

See [Running ROS2 Exercises](../../RUNNING_EXERCISES.md) for build commands, package names, executable names, and sample topic inputs.
