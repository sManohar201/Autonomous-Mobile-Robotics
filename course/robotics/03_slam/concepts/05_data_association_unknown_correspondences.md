# 4. Data Association (Unknown Correspondences)

Without known correspondences, the filter must match observations to known
landmarks. Two approaches:

### Maximum likelihood (NN-based)

For each new observation z, compute Mahalanobis distance to each known landmark:

```
dᵢ² = (z - hᵢ(x))ᵀ · Sᵢ⁻¹ · (z - hᵢ(x))    where Sᵢ = Hᵢ·P·Hᵢᵀ + R
```

- If min(dᵢ²) < χ²_threshold: associate with landmark i*
- Else: create new landmark

The χ² threshold (e.g., 5.99 for 95% confidence, 2 DOF) controls the
false-positive vs. false-negative tradeoff.

### Joint compatibility branch-and-bound (JCBB)

More expensive but handles simultaneous matching of multiple landmarks.
Used in production-grade SLAM systems.
