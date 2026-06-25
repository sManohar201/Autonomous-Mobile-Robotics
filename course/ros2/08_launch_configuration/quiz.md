# ROS2 Module 08 Quiz - Launch & Configuration

## Q1

Why should simulation-specific wiring stay out of production node logic?

<details><summary>Answer</summary>

The node should implement subsystem behavior. Launch files decide whether inputs come from hardware, bags, synthetic publishers, or simulation.

</details>

## Q2

What belongs in YAML instead of source code?

<details><summary>Answer</summary>

Robot- and environment-specific parameters such as topic names, expected rates, thresholds, frames, and timeouts.

</details>

## Q3

Why use launch arguments?

<details><summary>Answer</summary>

They let operators or higher-level launch files select configuration, namespaces, robot variants, and optional components.

</details>

