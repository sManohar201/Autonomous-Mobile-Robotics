# 7. Normal Distributions Transform (NDT)

NDT is an alternative to ICP that models the target cloud as a set of Gaussian
distributions over a voxel grid, then maximises the likelihood of source points:

```
score = Σᵢ pᵢᵀ · Σᵢ⁻¹ · pᵢ    where pᵢ = sᵢ - μcell, Σcell = covariance of target in cell
```

NDT advantages:
- More robust to outliers (Gaussian model vs. nearest-neighbour)
- Faster for dense 3D clouds (Autoware uses NDT for LiDAR localisation)
- No explicit correspondence needed
