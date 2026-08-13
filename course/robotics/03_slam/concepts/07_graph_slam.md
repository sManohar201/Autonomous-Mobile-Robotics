# 6. Graph SLAM

Graph SLAM decouples front-end (odometry/scan matching) from back-end
(global pose optimisation).

```
Front-end:
  pose graph nodes:   x₁, x₂, ..., xₜ
  odometry edges:     (xᵢ, xᵢ₊₁, Δx, Σ)
  loop closure edges: (xᵢ, xⱼ, Δx, Σ)

Back-end:
  minimise: Σ eᵢⱼᵀ · Ωᵢⱼ · eᵢⱼ    where eᵢⱼ = f(xᵢ, xⱼ) - Δxᵢⱼ
  solution: sparse linear system (Cholesky or Conjugate Gradient)
```

The back-end is a sparse nonlinear least squares problem. Libraries:
- **g2o** — used by Cartographer, RTAB-Map
- **GTSAM** — factor graph formulation
- **iSAM2** — incremental smoothing (best for online SLAM)

### Toy 1D pose graph

```cpp
struct Edge { int from, to; double delta, information; };

// Minimise Σ info_ij * (x[j] - x[i] - delta_ij)²
// Gradient descent step:
for (const auto& e : edges) {
    const double err = states[e.to] - states[e.from] - e.delta;
    const double grad = 2.0 * e.information * err;
    states[e.from] += lr * grad;
    states[e.to]   -= lr * grad;
}
// Fix gauge: pin states[0] = 0
```
