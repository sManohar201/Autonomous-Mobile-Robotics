# ROS2 Module 01 - ROS2 Foundations

## Goal

Build a working mental model of ROS2 as a distributed graph of processes. The point of this chapter is to understand how nodes communicate and how those communication choices shape system architecture.

## The ROS2 Graph

The graph is the runtime network of nodes, topics, services, actions, parameters, and names. A field robot is not one program; it is a coordinated set of processes with explicit communication contracts.

You should be able to answer:

- Which node owns this data?
- Which topic carries it?
- What is the message type?
- What happens if the publisher stops?
- What happens if the subscriber starts late?

## Nodes

A node is a unit of responsibility. Good production nodes have narrow ownership:

- One node owns a sensor driver.
- One node owns a filter.
- One node owns a planner.
- One node owns robot health aggregation.

Avoid "god nodes" that subscribe to everything and decide everything. They become difficult to test, restart, and reason about.

## Topics

Topics are streaming data channels. Use them for facts that change over time:

- `/odom`
- `/imu/data`
- `/front_laser/scan`
- `/robot/status`

Topics should be named for the data they carry, not for who consumes them.

## Services

Services are request/response RPC. Use them for bounded operations:

- reset a filter
- request current health snapshot
- clear a costmap

Do not use services for long-running goals or streaming sensor data.

## Actions

Actions are long-running goals with feedback and cancellation:

- navigate to pose
- dock
- perform scan routine
- execute inspection mission

If cancellation matters, an action is usually a better fit than a service.

## Parameters

Parameters configure node behavior. Production parameter handling requires declared parameters, defaults, validation, clear failure behavior, and launch/YAML integration.

Do not silently accept invalid configuration. Invalid parameters should fail fast or put the node into a safe inactive state.

## Launch Files

Launch files describe system composition: which nodes run, with what names, namespaces, parameters, remaps, and conditions.

Production launch files should separate robot-independent logic, robot-specific configuration, environment-specific choices, and simulation-only wiring.

## Capstone Direction

The first capstone is a `robot_monitor` prototype:

- subscribes to status-like inputs
- tracks freshness
- exposes expected rates as parameters
- publishes a simple robot status summary

At this stage it can use synthetic test publishers. Gazebo is not required yet.

