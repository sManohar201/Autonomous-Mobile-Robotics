# ROS2 Module 07 Quiz - State Estimation Architecture

## Q1

Why keep the filter core independent from ROS message types?

<details><summary>Answer</summary>

It makes the estimator easier to unit test, reuse, and reason about. ROS-facing adapters handle message conversion while the core owns the math.

</details>

## Q2

What does covariance communicate?

<details><summary>Answer</summary>

Uncertainty. Consumers use covariance to understand confidence and fuse estimates correctly.

</details>

## Q3

Why should reset be a controlled service rather than a direct variable write?

<details><summary>Answer</summary>

Reset changes estimator state and can affect robot behavior. It needs validation, logging, diagnostics, and predictable side effects.

</details>

