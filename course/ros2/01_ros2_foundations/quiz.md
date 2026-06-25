# ROS2 Module 01 Quiz - ROS2 Foundations

## Q1

When should a ROS2 interface be a topic instead of a service?

<details><summary>Answer</summary>

Use a topic for streaming data or state that changes over time, such as odometry, IMU, scan data, or robot status. Services are for bounded request/response operations.

</details>

## Q2

Why is a "god node" a bad production architecture?

<details><summary>Answer</summary>

It mixes responsibilities, becomes hard to test, and creates large failure domains. Narrow nodes can be restarted, tested, monitored, and replaced independently.

</details>

## Q3

What should happen when a required parameter is invalid?

<details><summary>Answer</summary>

The node should reject the configuration clearly. Depending on the lifecycle model, it should fail configuration, remain inactive, or shut down rather than silently running with invalid behavior.

</details>

## Q4

What makes actions different from services?

<details><summary>Answer</summary>

Actions support long-running goals, feedback, result reporting, and cancellation. Services are simpler request/response interactions.

</details>

## Q5

Why should topic names describe data rather than consumers?

<details><summary>Answer</summary>

Data-oriented names keep interfaces stable when consumers change. A topic like `/odom` can feed localization, monitoring, logging, and planning without being tied to one consumer.

</details>

