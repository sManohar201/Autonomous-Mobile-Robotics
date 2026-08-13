# 4. The Extended Kalman Filter (EKF)

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
