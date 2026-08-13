# 7. Sensor Noise Models

| Sensor | Noise model | Typical σ |
|---|---|---|
| GPS (outdoor) | White noise on x, y | 1–5 m (standard), 0.3 m (RTK) |
| GPS (indoor) | N/A — doesn't work | — |
| IMU gyro | White noise + bias drift | 0.01 rad/s white, 0.001 rad/s/√s bias |
| Wheel odometry | Proportional to distance | 1–5% of distance travelled |
| 2D LiDAR (ICP) | Scan-match dependent | 0.01–0.05 m / 0.01 rad |

### Process noise Q

Q captures unmodelled dynamics: wheel slip, uneven terrain, wind.

For a differential drive robot:

```cpp
// Q diagonal: [var_px, var_py, var_theta]
// Slip adds ~1% uncertainty in x,y per timestep; heading drifts ~0.1 deg/s
Q = diag(0.01*dt, 0.01*dt, 0.001*dt);
```

Q is tuned empirically: start large (trust measurements), reduce until the
filter diverges under pure prediction, then increase slightly.
