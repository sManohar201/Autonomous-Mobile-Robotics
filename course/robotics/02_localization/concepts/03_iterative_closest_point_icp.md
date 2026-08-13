# 2. Iterative Closest Point (ICP)

ICP finds the rigid-body transform (R, t) that best aligns a source point cloud
to a target cloud by iteratively minimising point-to-point distances.

### Algorithm

```
Input:  source cloud S, target cloud T, initial guess T₀
Output: transform (R, t) such that R·S + t ≈ T

Repeat until convergence:
  1. Correspondence:   for each point sᵢ ∈ S, find nearest neighbour tᵢ ∈ T
  2. Transformation:   compute (R, t) minimising Σ ||R·sᵢ + t - tᵢ||²
  3. Apply:            S ← R·S + t
  4. Check:            if RMS change < ε, stop
```

### Step 2: Optimal rotation via SVD

Given matched pairs {(sᵢ, tᵢ)}:

```
Compute centroids:  s̄ = mean(sᵢ),  t̄ = mean(tᵢ)
Demeaned:           sᵢ' = sᵢ - s̄,  tᵢ' = tᵢ - t̄
Cross-covariance:   H = Σ sᵢ' · tᵢ'ᵀ
SVD:                H = U · Σ · Vᵀ
Rotation:           R = V · Uᵀ
Translation:        t = t̄ - R · s̄
```

```cpp
// Compute cross-covariance H
Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
for (size_t i = 0; i < src.size(); ++i) {
    H += (src[i] - src_mean) * (tgt[i] - tgt_mean).transpose();
}
// SVD → rotation
Eigen::JacobiSVD<Eigen::Matrix2d> svd(H,
    Eigen::ComputeFullU | Eigen::ComputeFullV);
Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();
// Correct reflection: det(R) must be +1
if (R.determinant() < 0) {
    Eigen::Matrix2d diag = Eigen::Matrix2d::Identity();
    diag(1, 1) = -1.0;
    R = svd.matrixV() * diag * svd.matrixU().transpose();
}
Eigen::Vector2d t = tgt_mean - R * src_mean;
```

### ICP failure modes

| Issue | Cause | Fix |
|---|---|---|
| Local minimum | Bad initial guess | Use odometry for initial guess, coarse-to-fine |
| Slow convergence | Dense clouds | Subsample source |
| Outlier sensitivity | Mismatched points | Trim outliers (TrimmedICP), use robust cost |
| Planar degeneracy | Corridor — no side walls | Add point-to-plane variant |

### Point-to-plane ICP

Instead of minimising point-to-point distance, minimise the component along the
surface normal of the target:

```
cost = Σ (nᵢ · (R·sᵢ + t - tᵢ))²    where nᵢ is the target surface normal
```

Converges in ~3× fewer iterations than point-to-point in structured environments.
