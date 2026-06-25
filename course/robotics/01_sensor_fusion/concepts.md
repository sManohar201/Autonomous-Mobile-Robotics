# Robotics Module 01 — Sensor Fusion

## Goal

Understand the mathematics behind combining noisy sensor measurements into a
single best estimate of robot state. Master the Extended Kalman Filter (EKF)
used in production localization stacks (robot_localization, Nav2), and
understand where the Unscented Kalman Filter (UKF) improves on it.

---

## 1. The State Estimation Problem

A mobile robot has state **x** — position, orientation, velocity — that we cannot
measure directly. Instead we have:
- A **process model** f(**x**, **u**) — how state evolves given control input **u**
- **Sensor measurements** **z** — noisy observations related to state

Goal: compute the **posterior** p(**x** | **z₁:t**, **u₁:t**) — the probability
distribution over robot state given all past measurements and controls.

For linear systems with Gaussian noise, this posterior is exactly Gaussian, and
the Kalman Filter is the optimal estimator. For nonlinear systems (any real robot),
we use approximations: EKF or UKF.

---

## 2. Probability Fundamentals

### Gaussian distribution

A scalar Gaussian: x ~ N(μ, σ²) has PDF:

```
p(x) = (1 / √(2πσ²)) · exp(-(x-μ)² / (2σ²))
```

For a state vector **x** ∈ ℝⁿ, the multivariate Gaussian N(**μ**, **P**):

```
p(x) = (1 / √((2π)ⁿ|P|)) · exp(-½ (x-μ)ᵀ P⁻¹ (x-μ))
```

**μ** is the mean (our best estimate), **P** is the covariance matrix (our
uncertainty). The diagonal of **P** gives variance per dimension; off-diagonal
elements capture correlations.

### Why Gaussian?

1. Closed under linear transformation: if **x** ~ N(**μ**, **P**), then
   **Ax** ~ N(**Aμ**, **APAᵀ**)
2. Product of two Gaussians is Gaussian (Bayes update in closed form)
3. Kalman Filter is optimal for linear Gaussian systems (MMSE estimator)

---

## 3. The Kalman Filter (Linear Case)

State: **x** ∈ ℝⁿ  
Measurement: **z** ∈ ℝᵐ  

**State transition:**  `xₜ = F·xₜ₋₁ + B·uₜ + wₜ`,   wₜ ~ N(**0**, **Q**)  
**Measurement model:** `zₜ = H·xₜ + vₜ`,              vₜ ~ N(**0**, **R**)

### Predict step

```
x̂ₜ⁻ = F · x̂ₜ₋₁ + B · uₜ          (predicted mean)
Pₜ⁻  = F · Pₜ₋₁ · Fᵀ + Q          (predicted covariance)
```

**Q** is process noise — how much the world changes unpredictably between
measurements. Large Q = trust measurements more. Small Q = trust the model more.

### Update step

```
Kₜ   = Pₜ⁻ · Hᵀ · (H · Pₜ⁻ · Hᵀ + R)⁻¹   (Kalman gain)
x̂ₜ  = x̂ₜ⁻ + Kₜ · (zₜ - H · x̂ₜ⁻)          (updated mean)
Pₜ   = (I - Kₜ · H) · Pₜ⁻                    (updated covariance)
```

**K** is the Kalman gain — how much we trust the measurement vs. the prediction.
- K → 0: large measurement noise R → ignore measurement, trust prediction
- K → H⁻¹: small measurement noise → trust measurement, ignore prediction

The innovation `(zₜ - H · x̂ₜ⁻)` is the residual — how wrong the prediction was.

---

## 4. The Extended Kalman Filter (EKF)

Real robots have nonlinear dynamics and nonlinear sensor models:
- Process: **xₜ** = **f**(**xₜ₋₁**, **uₜ**) + **w**
- Measurement: **zₜ** = **h**(**xₜ**) + **v**

The EKF linearises these functions at the current estimate using Jacobians:

```
Fₜ = ∂f/∂x |_{x̂ₜ₋₁}     (Jacobian of process model w.r.t. state)
Hₜ = ∂h/∂x |_{x̂ₜ⁻}      (Jacobian of measurement model w.r.t. state)
```

### EKF Predict

```
x̂ₜ⁻ = f(x̂ₜ₋₁, uₜ)           (nonlinear prediction)
Pₜ⁻  = Fₜ · Pₜ₋₁ · Fₜᵀ + Q   (linearised covariance propagation)
```

### EKF Update

```
Kₜ  = Pₜ⁻ · Hₜᵀ · (Hₜ · Pₜ⁻ · Hₜᵀ + R)⁻¹
x̂ₜ = x̂ₜ⁻ + Kₜ · (zₜ - h(x̂ₜ⁻))
Pₜ  = (I - Kₜ · Hₜ) · Pₜ⁻
```

The only change from the linear KF: f, h are called instead of F·x, H·x; and
the Jacobians Fₜ, Hₜ replace the constant matrices F, H.

---

## 5. 2D Robot State Model

For a ground robot with 2D position and heading:

```
x = [px, py, θ]ᵀ
```

### Process model (differential drive)

Given linear velocity v and angular velocity ω over timestep Δt:

```
px' = px + v · cos(θ) · Δt
py' = py + v · sin(θ) · Δt
θ'  = θ + ω · Δt
```

Process Jacobian Fₜ = ∂f/∂x:

```
    [1   0   -v·sin(θ)·Δt]
F = [0   1    v·cos(θ)·Δt]
    [0   0    1           ]
```

### GPS measurement model

GPS measures position directly:

```
z = [px, py]ᵀ
h(x) = [px, py]ᵀ

H = [1  0  0]
    [0  1  0]
```

### Odometry measurement model

Odometry measures differential changes:

```
z = [Δpx, Δpy, Δθ]ᵀ — incremental pose change
```

This uses the full state; H = I₃.

### IMU measurement model

IMU measures angular velocity ω directly:

```
z = ω
h(x) = θ_dot (rate of change of heading)
```

---

## 6. Worked Example: EKF Predict + Update (C++)

```cpp
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

struct EKF2D {
    Vec3 x;   // state: [px, py, theta]
    Mat3 P;   // covariance
    Mat3 Q;   // process noise
};

EKF2D predict(const EKF2D& ekf, double v, double omega, double dt) {
    const double theta = ekf.x(2);

    // Nonlinear state transition
    Vec3 x_pred;
    x_pred(0) = ekf.x(0) + v * std::cos(theta) * dt;
    x_pred(1) = ekf.x(1) + v * std::sin(theta) * dt;
    x_pred(2) = ekf.x(2) + omega * dt;

    // Jacobian of f w.r.t. x
    Mat3 F = Mat3::Identity();
    F(0, 2) = -v * std::sin(theta) * dt;
    F(1, 2) =  v * std::cos(theta) * dt;

    // Covariance prediction
    Mat3 P_pred = F * ekf.P * F.transpose() + ekf.Q;

    return {x_pred, P_pred, ekf.Q};
}

EKF2D update_gps(const EKF2D& ekf, double gps_x, double gps_y,
                 double sigma_gps) {
    // H = [1 0 0; 0 1 0]
    Eigen::Matrix<double, 2, 3> H = Eigen::Matrix<double, 2, 3>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * sigma_gps * sigma_gps;

    Eigen::Vector2d z(gps_x, gps_y);
    Eigen::Vector2d h = H * ekf.x;         // predicted measurement
    Eigen::Vector2d innov = z - h;          // innovation

    Eigen::Matrix2d S = H * ekf.P * H.transpose() + R;
    Eigen::Matrix<double, 3, 2> K = ekf.P * H.transpose() * S.inverse();

    Vec3 x_upd = ekf.x + K * innov;
    Mat3 P_upd = (Mat3::Identity() - K * H) * ekf.P;

    return {x_upd, P_upd, ekf.Q};
}

int main() {
    EKF2D ekf;
    ekf.x = Vec3(0, 0, 0);
    ekf.P = Mat3::Identity() * 0.1;
    ekf.Q = Mat3::Identity() * 0.01;

    // Predict: move forward 1 m/s for 0.1 s
    ekf = predict(ekf, 1.0, 0.0, 0.1);
    std::cout << "After predict: x=" << ekf.x.transpose() << "\n";
    std::cout << "P diag: " << ekf.P.diagonal().transpose() << "\n";

    // Update with GPS reading at (0.09, 0.0) — slightly off from prediction
    ekf = update_gps(ekf, 0.09, 0.0, 0.5);  // sigma_gps = 0.5 m
    std::cout << "After GPS update: x=" << ekf.x.transpose() << "\n";
    std::cout << "P diag: " << ekf.P.diagonal().transpose() << "\n";

    return 0;
}
```

Expected output (approximate):
```
After predict: x=0.1 0 0
P diag: 0.11 0.11 0.11
After GPS update: x=0.0997 -0.00012 0
P diag: 0.0284 0.0284 0.11
```

The GPS update reduced uncertainty in x and y (P diagonal smaller) but not in
theta (GPS doesn't observe heading directly).

---

## 7. Sensor Noise Models

| Sensor | Noise model | Typical σ |
|---|---|---|
| GPS (outdoor) | White noise on x, y | 1–5 m (standard), 0.3 m (RTK) |
| GPS (indoor) | N/A — doesn't work | — |
| IMU gyro | White noise + bias drift | 0.01 rad/s white, 0.001 rad/s/√s bias |
| Wheel odometry | Proportional to distance | 1–5% of distance travelled |
| 2D LiDAR (ICP) | Scan-match dependent | 0.01–0.05 m / 0.01 rad |

### Process noise Q

Q captures unmodelled dynamics: wheel slip, uneven terrain, wind.

For a differential drive robot:

```cpp
// Q diagonal: [var_px, var_py, var_theta]
// Slip adds ~1% uncertainty in x,y per timestep; heading drifts ~0.1 deg/s
Q = diag(0.01*dt, 0.01*dt, 0.001*dt);
```

Q is tuned empirically: start large (trust measurements), reduce until the
filter diverges under pure prediction, then increase slightly.

---

## 8. The Unscented Kalman Filter (UKF)

The EKF linearises f and h at a single point. For highly nonlinear systems,
this linearisation introduces large errors — the EKF can diverge.

The UKF instead propagates a carefully chosen set of **sigma points** through
the nonlinear functions and recomputes mean and covariance from the results.

### Sigma point selection (for state dimension n)

```
Sigma points: 2n+1 points
χ₀ = x̂
χᵢ = x̂ + (√((n+λ)P))ᵢ      for i = 1..n
χᵢ = x̂ - (√((n+λ)P))ᵢ₋ₙ   for i = n+1..2n

λ = α²(n + κ) - n
```

### UKF Predict

1. Generate 2n+1 sigma points from (x̂, P)
2. Propagate each through f: χᵢ* = f(χᵢ, u)
3. Recompute predicted mean and covariance:
```
x̂⁻ = Σ Wᵐᵢ · χᵢ*
P⁻  = Σ Wᶜᵢ · (χᵢ* - x̂⁻)(χᵢ* - x̂⁻)ᵀ + Q
```

### When UKF beats EKF

- Highly nonlinear process models (large Δt, fast turning)
- Sensors with nonlinear measurement functions (bearing-only, range-bearing)
- Initial uncertainty is large relative to the nonlinearity

### When EKF is sufficient

- Slow-moving robots with high-rate sensors (small Δt → near-linear)
- Linear or mildly nonlinear sensor models
- Most production navigation stacks use EKF for speed

> **robot_localization** (the Nav2 default) uses EKF and optionally UKF.
> EKF is adequate for differential drive robots at typical speeds.

---

## 9. Multi-Sensor Fusion Strategy

### Sensor selection by availability

```
Primary: Wheel odometry (always available, low latency, drifts)
Secondary: IMU (high rate, good for angular velocity, gyro bias)
Tertiary: GPS (absolute, low rate, not indoor)
Quaternary: LiDAR ICP (computationally heavy, very accurate short-term)
```

### Fusion timing

| Update | Rate | Triggers |
|---|---|---|
| Predict | 50–100 Hz | IMU callback or control command |
| Odom update | 50 Hz | Odometry callback |
| GPS update | 5–10 Hz | GPS fix callback |
| LiDAR update | 1–5 Hz | ICP convergence callback |

### Gating (outlier rejection)

Reject measurements that are too far from the prediction (Mahalanobis distance):

```
d² = (z - h(x̂))ᵀ · S⁻¹ · (z - h(x̂))    where S = H·P·Hᵀ + R
```

If d² > χ²_threshold (e.g., 9.21 for 95% confidence with 2 DOF), reject.
This prevents GPS multipath or ICP false convergences from corrupting the filter.

```cpp
double mahalanobis_sq(const Eigen::VectorXd& innov,
                      const Eigen::MatrixXd& S) {
    return innov.transpose() * S.inverse() * innov;
}
// Reject if > 9.21 (95% chi-squared, 2 DOF)
if (mahalanobis_sq(innov, S) > 9.21) {
    RCLCPP_WARN(logger, "Rejected GPS: Mahalanobis=%.2f", d);
    return;
}
```

---

## 10. Covariance Interpretation

The covariance matrix **P** tells you:
- Diagonal: variance per state dimension (σ² = uncertainty squared)
- Off-diagonal: how uncertainties correlate

For a 2D position estimate:
```
P = [0.01   0.005]
    [0.005  0.02 ]

σ_x = √0.01  = 0.1 m   (x uncertainty)
σ_y = √0.02  = 0.14 m  (y uncertainty)
correlation(x,y) = 0.005 / (0.1 · 0.14) ≈ 0.36
```

**Covariance divergence:** if `P.trace()` grows unboundedly, the filter
is diverging — either Q is too small, the model is wrong, or all measurements
are being rejected.

---

## 11. Interview Questions

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
