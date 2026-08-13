# 3. The Kalman Filter (Linear Case)

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
