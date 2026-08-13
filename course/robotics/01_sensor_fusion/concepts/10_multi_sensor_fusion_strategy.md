# 9. Multi-Sensor Fusion Strategy

### Sensor selection by availability

```
Primary: Wheel odometry (always available, low latency, drifts)
Secondary: IMU (high rate, good for angular velocity, gyro bias)
Tertiary: GPS (absolute, low rate, not indoor)
Quaternary: LiDAR ICP (computationally heavy, very accurate short-term)
```

### Fusion timing

| Update | Rate | Triggers |
|---|---|---|
| Predict | 50–100 Hz | IMU callback or control command |
| Odom update | 50 Hz | Odometry callback |
| GPS update | 5–10 Hz | GPS fix callback |
| LiDAR update | 1–5 Hz | ICP convergence callback |

### Gating (outlier rejection)

Reject measurements that are too far from the prediction (Mahalanobis distance):

```
d² = (z - h(x̂))ᵀ · S⁻¹ · (z - h(x̂))    where S = H·P·Hᵀ + R
```

If d² > χ²_threshold (e.g., 9.21 for 95% confidence with 2 DOF), reject.
This prevents GPS multipath or ICP false convergences from corrupting the filter.

```cpp
double mahalanobis_sq(const Eigen::VectorXd& innov,
                      const Eigen::MatrixXd& S) {
    return innov.transpose() * S.inverse() * innov;
}
// Reject if > 9.21 (95% chi-squared, 2 DOF)
if (mahalanobis_sq(innov, S) > 9.21) {
    RCLCPP_WARN(logger, "Rejected GPS: Mahalanobis=%.2f", d);
    return;
}
```
