# 10. Covariance Interpretation

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
