# ROS2 Module 10 Quiz - Gazebo Integration

## Q1

Why keep bridge configuration out of node logic?

<details><summary>Answer</summary>

The node should consume ROS2 interfaces. Whether those interfaces come from hardware, bags, synthetic publishers, or Gazebo is a deployment concern.

</details>

## Q2

What common issue does `use_sim_time` introduce?

<details><summary>Answer</summary>

Nodes can start before `/clock` is available or experience time jumps, which affects timers, stale checks, and TF.

</details>

## Q3

Why test with synthetic publishers before Gazebo?

<details><summary>Answer</summary>

Synthetic publishers isolate node logic. Gazebo adds startup, timing, bridge, and model complexity.

</details>

