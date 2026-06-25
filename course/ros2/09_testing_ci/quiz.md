# ROS2 Module 09 Quiz - Testing & CI

## Q1

Why keep pure logic outside ROS nodes when possible?

<details><summary>Answer</summary>

Pure logic is easier and faster to unit test without ROS middleware, executors, timing, or launch machinery.

</details>

## Q2

What is bag replay useful for?

<details><summary>Answer</summary>

Regression testing with realistic recorded data.

</details>

## Q3

Why separate fast tests from long simulation tests?

<details><summary>Answer</summary>

Fast tests should run frequently and catch most regressions. Long simulation tests are valuable but too slow/flaky for every edit loop.

</details>

