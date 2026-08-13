# 8. The Unscented Kalman Filter (UKF)

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
