# 2. Probability Fundamentals

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
