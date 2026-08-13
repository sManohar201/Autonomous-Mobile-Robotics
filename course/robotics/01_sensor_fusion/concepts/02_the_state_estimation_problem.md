# 1. The State Estimation Problem

A mobile robot has state **x** — position, orientation, velocity — that we cannot
measure directly. Instead we have:
- A **process model** f(**x**, **u**) — how state evolves given control input **u**
- **Sensor measurements** **z** — noisy observations related to state

Goal: compute the **posterior** p(**x** | **z₁:t**, **u₁:t**) — the probability
distribution over robot state given all past measurements and controls.

For linear systems with Gaussian noise, this posterior is exactly Gaussian, and
the Kalman Filter is the optimal estimator. For nonlinear systems (any real robot),
we use approximations: EKF or UKF.
