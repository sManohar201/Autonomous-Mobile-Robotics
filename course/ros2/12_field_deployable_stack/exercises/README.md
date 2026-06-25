# Exercises - Field Deployable Stack

## Moderate Exercises

### Exercise 1 - Stack Diagram

Draw the full robot software stack, including packages, nodes, topics, services, actions, launch files, and diagnostics.

### Exercise 2 - Degraded Mode Table

Create a degraded mode table for sensor, localization, planning, and control faults.

### Exercise 3 - Deployment Checklist

Write a pre-field checklist for configuration, launch, topic health, diagnostics, and bag recording.

## Hard Exercises

### Hard Exercise 1 - Production Stack Skeleton

Assemble the course subsystems into a deployable skeleton with:

- bringup launch
- robot config
- simulation config
- monitoring
- lifecycle flow
- diagnostics
- bagging policy

### Hard Exercise 2 - Field Failure Review

Given a failure report, trace which diagnostics, logs, bags, and launch configs you would inspect first. Propose a code or configuration change and a regression test.

## Starter and Reference Package

- Starter package: `exercises/field_deployable_stack_lab/`
- Reference package: `../answers/field_deployable_stack_lab/`
- Moderate target: `deployment_checklist_node`
- Hard target: `degraded_mode_manager_hard`

## How to Run

See [Running ROS2 Exercises](../../RUNNING_EXERCISES.md) for build commands, package names, executable names, and sample topic inputs.
