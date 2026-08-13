# 1. The Localization Problem

Localization: given a known map **m** and sensor history **z₁:t**, **u₁:t**,
compute p(**x**t | **m**, **z₁:t**, **u₁:t**).

Three sub-problems with increasing difficulty:

| Problem | Known? | Unknown? |
|---|---|---|
| Tracking | Initial pose | Future poses |
| Global localization | Map only | Initial pose + future poses |
| Kidnapped robot | Map only | Robot was moved, must re-localize |

EKF (Module 01) solves tracking. Particle filters solve all three.
