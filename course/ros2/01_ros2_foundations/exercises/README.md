# Exercises - ROS2 Foundations

## Moderate Exercises

### Exercise 1 - Graph Inspection

Run a minimal talker/listener pair and document node names, topic names, message types, publisher count, and subscriber count.

Expected commands:

```bash
ros2 node list
ros2 topic list
ros2 topic info /topic_name
ros2 interface show <message_type>
```

### Exercise 2 - Parameterized Status Publisher

Create a node that publishes `/robot/status_text` at a configurable rate.

Starter package:

```text
exercises/robot_status_publisher/
```

Reference implementation:

```text
../answers/robot_status_publisher/
```

Requirements:

- declare `rate_hz`
- reject `rate_hz <= 0`
- publish a string message
- log startup configuration

### Exercise 3 - Reset Service

Add a service that resets an internal counter.

Requirements:

- counter increments every publish
- service resets counter to zero
- service response reports success

## Hard Exercises

### Hard Exercise 1 - Interface Decision Review

Given these robot interactions, decide topic, service, or action and justify each:

- stream IMU samples
- reset localization
- navigate to a pose
- report current battery state
- run a 30-second inspection routine

### Hard Exercise 2 - `robot_monitor` Prototype

Build a `robot_monitor` node with parameters for expected topic names, expected rate, and stale timeout. Track last-seen timestamps, publish a summary status, and log stale inputs.

Do not use Gazebo yet. Use synthetic publishers so the monitoring logic is deterministic and testable.

Starter package:

```text
exercises/robot_monitor_prototype/
```

Reference implementation:

```text
../answers/robot_monitor_prototype/
```

## How to Run

See [Running ROS2 Exercises](../../RUNNING_EXERCISES.md) for build commands, package names, executable names, and sample topic inputs.

- Status publisher starter: `ros2 run robot_status_publisher robot_status_publisher`
- Status publisher answer: `ros2 run robot_status_publisher_answer robot_status_publisher_answer`
- Monitor prototype starter: `ros2 run robot_monitor_prototype robot_monitor_prototype`
- Monitor prototype answer: `ros2 run robot_monitor_prototype_answer robot_monitor_prototype_answer`
