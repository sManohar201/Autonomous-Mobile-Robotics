# Running ROS2 Exercises

These exercises are real ROS2 packages. Each module provides a starter package under `exercises/` and a reference package under `answers/`.

The examples below assume ROS2 Jazzy is installed at `/opt/ros/jazzy`.

## Build One Exercise Package

Run from the repository root:

```bash
source /opt/ros/jazzy/setup.bash

PACKAGE_PATH=course/ros2/07_state_estimation_architecture/exercises/state_estimation_architecture_lab
BUILD_ID=ros2_m07_state_estimation

colcon --log-base /tmp/${BUILD_ID}_log build \
  --base-paths "${PACKAGE_PATH}" \
  --build-base /tmp/${BUILD_ID}_build \
  --install-base /tmp/${BUILD_ID}_install

source /tmp/${BUILD_ID}_install/setup.bash
ros2 run state_estimation_architecture_lab measurement_adapter
```

To run the reference answer instead, change `PACKAGE_PATH` to the matching `answers/` package and run the answer executable:

```bash
PACKAGE_PATH=course/ros2/07_state_estimation_architecture/answers/state_estimation_architecture_lab
BUILD_ID=ros2_m07_state_estimation_answer

colcon --log-base /tmp/${BUILD_ID}_log build \
  --base-paths "${PACKAGE_PATH}" \
  --build-base /tmp/${BUILD_ID}_build \
  --install-base /tmp/${BUILD_ID}_install

source /tmp/${BUILD_ID}_install/setup.bash
ros2 run state_estimation_architecture_lab_answer measurement_adapter_answer
```

## Build All Answers

```bash
source /opt/ros/jazzy/setup.bash

colcon --log-base /tmp/ros2_course_answers_log build \
  --base-paths course/ros2/*/answers/* \
  --build-base /tmp/ros2_course_answers_build \
  --install-base /tmp/ros2_course_answers_install

source /tmp/ros2_course_answers_install/setup.bash
```

## Package and Executable Index

| Module | Starter package | Starter executables | Answer package | Answer executables |
| --- | --- | --- | --- | --- |
| 01 Foundations | `robot_status_publisher` | `robot_status_publisher` | `robot_status_publisher_answer` | `robot_status_publisher_answer` |
| 01 Foundations | `robot_monitor_prototype` | `robot_monitor_prototype` | `robot_monitor_prototype_answer` | `robot_monitor_prototype_answer` |
| 02 Package Architecture | `package_architecture_lab` | `package_boundary_review`, `dependency_audit_hard` | `package_architecture_lab_answer` | `package_boundary_review_answer`, `dependency_audit_hard_answer` |
| 03 Production Nodes | `production_nodes_lab` | `parameter_validation_node`, `lifecycle_processor_hard` | `production_nodes_lab_answer` | `parameter_validation_node_answer`, `lifecycle_processor_hard_answer` |
| 04 Communication Design | `communication_design_lab` | `qos_status_publisher`, `command_api_hard` | `communication_design_lab_answer` | `qos_status_publisher_answer`, `command_api_hard_answer` |
| 05 Executors & Concurrency | `executors_concurrency_lab` | `snapshot_processor`, `concurrent_pipeline_hard` | `executors_concurrency_lab_answer` | `snapshot_processor_answer`, `concurrent_pipeline_hard_answer` |
| 06 Sensor Integration | `sensor_integration_lab` | `frame_validator`, `sensor_health_hard` | `sensor_integration_lab_answer` | `frame_validator_answer`, `sensor_health_hard_answer` |
| 07 State Estimation | `state_estimation_architecture_lab` | `measurement_adapter`, `localization_shell_hard` | `state_estimation_architecture_lab_answer` | `measurement_adapter_answer`, `localization_shell_hard_answer` |
| 08 Launch & Configuration | `launch_configuration_lab` | `config_echo_node`, `variant_bringup_hard` | `launch_configuration_lab_answer` | `config_echo_node_answer`, `variant_bringup_hard_answer` |
| 09 Testing & CI | `testing_ci_lab` | `freshness_logic_node`, `test_report_hard` | `testing_ci_lab_answer` | `freshness_logic_node_answer`, `test_report_hard_answer` |
| 10 Gazebo Integration | `gazebo_integration_lab` | `sim_time_check`, `gazebo_monitor_hard` | `gazebo_integration_lab_answer` | `sim_time_check_answer`, `gazebo_monitor_hard_answer` |
| 11 Observability & Operations | `observability_operations_lab` | `diagnostic_aggregator`, `fault_report_hard` | `observability_operations_lab_answer` | `diagnostic_aggregator_answer`, `fault_report_hard_answer` |
| 12 Field Deployable Stack | `field_deployable_stack_lab` | `deployment_checklist_node`, `degraded_mode_manager_hard` | `field_deployable_stack_lab_answer` | `deployment_checklist_node_answer`, `degraded_mode_manager_hard_answer` |

## Feeding Example Inputs

Many nodes publish timer-based summaries and can run by themselves. Subscriber-based nodes are easier to inspect if you publish sample messages from a second terminal after sourcing the same install space.

String status input:

```bash
ros2 topic pub /subsystem/status std_msgs/msg/String "{data: 'localization:DEGRADED'}" -r 1
```

Freshness test input:

```bash
ros2 topic pub /test/input std_msgs/msg/String "{data: 'heartbeat'}" -r 2
```

Odometry input:

```bash
ros2 topic pub /odom nav_msgs/msg/Odometry \
  "{header: {frame_id: 'odom'}, pose: {pose: {position: {x: 1.0, y: 2.0}, orientation: {w: 1.0}}}}" -r 5
```

Gazebo clock input without Gazebo:

```bash
ros2 topic pub /clock rosgraph_msgs/msg/Clock "{clock: {sec: 1, nanosec: 0}}" -r 10
```

## Common Debug Commands

```bash
ros2 pkg executables <package_name>
ros2 node list
ros2 topic list
ros2 topic echo /operator/health
ros2 param list
ros2 param get /config_echo_node robot_name
```
