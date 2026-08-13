# 3. EKF SLAM (Known Correspondences)

The EKF SLAM state vector augments robot pose with landmark positions:

```
x = [px, py, θ, lx1, ly1, lx2, ly2, ..., lxN, lyN]
```

For N landmarks, dimension is 3 + 2N.

### State transition

Only the robot portion of the state changes during motion:

```
F = block_diag(F_robot, I_{2N})   (landmark positions don't move)
```

The covariance update `P' = F·P·Fᵀ + Q` expands the uncertainty in the
robot-landmark cross-correlations — a key feature of EKF SLAM.

### Observation model for landmark i

Robot at (px, py, θ), landmark at (lxi, lyi):

```
expected_range   = √((lxi-px)² + (lyi-py)²)
expected_bearing = atan2(lyi-py, lxi-px) - θ

h(x) = [expected_range, expected_bearing]ᵀ
```

Jacobian Hᵢ (2 × (3+2N)) — nonzero only in robot and landmark-i columns:

```cpp
const double dx = lx - px, dy = ly - py;
const double r2 = dx*dx + dy*dy, r = std::sqrt(r2);

// Jacobian w.r.t. robot state [px, py, θ]
H.block<2,3>(0, 0) = {
    {-dx/r,  -dy/r,   0},
    { dy/r2, -dx/r2, -1}
};
// Jacobian w.r.t. landmark i state [lxi, lyi]
H.block<2,2>(0, 3 + 2*i) = {
    {dx/r,   dy/r  },
    {-dy/r2, dx/r2 }
};
```

### Adding a new landmark

When a landmark is observed for the first time, initialise it:

```cpp
state(3 + 2*n) = state(0) + range * std::cos(bearing + state(2));
state(3 + 2*n+1) = state(1) + range * std::sin(bearing + state(2));

// Augment covariance — new landmark initially uncertain
P_new = block_diag(P_old, J_init * R_landmark * J_init.transpose());
```
