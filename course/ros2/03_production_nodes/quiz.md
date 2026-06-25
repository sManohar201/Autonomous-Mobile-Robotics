# ROS2 Module 03 Quiz - Production Nodes

## Q1

Why validate parameters before activation?

<details><summary>Answer</summary>

Activation starts runtime behavior. Invalid configuration should be caught before the node subscribes, publishes, commands hardware, or reports itself as ready.

</details>

## Q2

When should a node use lifecycle semantics?

<details><summary>Answer</summary>

When configuration, activation, deactivation, cleanup, and controlled startup/shutdown matter. This is common for drivers, filters, controllers, and field-deployable subsystems.

</details>

## Q3

What is the difference between logs and diagnostics?

<details><summary>Answer</summary>

Logs are human-readable event records. Diagnostics are structured health/status outputs meant for monitoring and automation.

</details>

## Q4

What should a node do when an input topic becomes stale?

<details><summary>Answer</summary>

It should detect the stale condition, update diagnostics/status, and move to a degraded or safe behavior depending on the subsystem.

</details>

## Q5

Why should callbacks avoid doing too much work directly?

<details><summary>Answer</summary>

Long callbacks can block other callbacks, increase latency, and make executor behavior harder to reason about. Work should be bounded or moved to controlled processing paths.

</details>

