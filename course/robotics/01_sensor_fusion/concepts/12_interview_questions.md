# 11. Interview Questions

**Q: Why use EKF instead of the basic Kalman Filter?**
A: Real robot dynamics and sensor models are nonlinear. The basic KF assumes
f and h are linear; EKF linearises them at the current estimate via Jacobians.

**Q: What happens if you set R too small?**
A: The filter trusts measurements too much — noisy sensor readings corrupt the
state estimate. The Kalman gain K approaches H⁻¹, ignoring the process model.

**Q: What happens if you set Q too small?**
A: The filter trusts the process model too much. Real disturbances (wheel slip,
wind) accumulate without correction. The covariance doesn't grow and the filter
rejects valid measurements as outliers.

**Q: What's the difference between EKF and UKF?**
A: EKF linearises nonlinear functions using Jacobians (first-order Taylor).
UKF uses sigma points to capture higher-order statistics without derivatives.
UKF is more accurate for strongly nonlinear systems; EKF is faster and simpler.

**Q: How do you fuse GPS + odometry in robot_localization?**
A: Configure `ekf_node` with two sources. Odometry provides [x, y, yaw] at
50 Hz for continuous tracking. GPS provides [x, y] at 10 Hz for absolute
correction. EKF merges them weighted by their covariances.

**Q: What is the innovation and why does it matter?**
A: `z - h(x̂)` is the innovation — how much the actual measurement differs from
the predicted measurement. If innovations are consistently large, either the
sensor model is wrong, the noise matrix R is wrong, or the sensor is faulty.
Monitoring innovation statistics is a key diagnostic.
