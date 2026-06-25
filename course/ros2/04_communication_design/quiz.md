# ROS2 Module 04 Quiz - Communication Design

## Q1

When is best-effort QoS acceptable?

<details><summary>Answer</summary>

For high-rate sensor streams where the newest data is more important than receiving every sample, such as laser scans or camera frames.

</details>

## Q2

Why include frame IDs in spatial messages?

<details><summary>Answer</summary>

Frame IDs define the coordinate frame of the data, allowing TF2 and consumers to interpret the measurement correctly.

</details>

## Q3

Why should nodes support namespaces?

<details><summary>Answer</summary>

Namespaces allow reuse across multiple robots, multiple sensors, and different deployment configurations without changing code.

</details>

## Q4

What is transient local durability useful for?

<details><summary>Answer</summary>

It lets late subscribers receive the last published value, useful for static or slowly changing state.

</details>

## Q5

Why are actions better than services for navigation goals?

<details><summary>Answer</summary>

Navigation is long-running and needs feedback, result reporting, and cancellation. Actions provide those semantics.

</details>

