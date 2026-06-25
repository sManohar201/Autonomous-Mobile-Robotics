# ROS2 Module 04 - Communication Design

## Goal

Choose ROS2 communication mechanisms intentionally: topics, services, actions, QoS, namespaces, and remapping are architecture decisions.

## Topics vs Services vs Actions

Use topics for streams, services for bounded request/response operations, and actions for long-running goals with feedback and cancellation.

## QoS

QoS controls delivery semantics. Important policies include reliability, durability, history, and depth.

Sensor streams often use best effort. Commands and critical state may require reliable delivery.

## Namespaces

Namespaces allow multiple robots or subsystem instances to coexist.

```text
/robot1/odom
/robot2/odom
```

Design nodes so namespace and remapping work cleanly.

## Message Design

Messages are long-lived contracts. Good messages have clear units, timestamps when needed, frame IDs for spatial data, and unambiguous field names.

## Capstone Direction

Design a command and telemetry API for a robot subsystem with topics, services, actions, and QoS choices.

