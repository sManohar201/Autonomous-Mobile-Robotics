# ROS2 Module 06 Quiz - Sensor Integration

## Q1

What should a message timestamp represent?

<details><summary>Answer</summary>

The time the data was measured, not the time it happened to be processed by a subscriber.

</details>

## Q2

Why is frame ID validation important?

<details><summary>Answer</summary>

A consumer must know the coordinate frame of spatial data. Wrong or missing frames can invalidate downstream transforms and estimates.

</details>

## Q3

Why is topic existence not enough for health?

<details><summary>Answer</summary>

A topic may exist but publish too slowly, stop intermittently, use wrong frames, or carry stale timestamps.

</details>

## Q4

What does `use_sim_time` affect?

<details><summary>Answer</summary>

It makes the node use the `/clock` simulation time source instead of wall time.

</details>

## Q5

Why introduce Gazebo only after core ROS2 architecture?

<details><summary>Answer</summary>

Simulation adds time, bridge, model, and sensor complexity. Students should first understand ROS2 interfaces, QoS, lifecycle, and testing without simulation noise.

</details>

