# 7. Sensor Integration Patterns

### Pattern A — Frame validator

Validate every incoming message has the expected frame ID. Parameterize so the
same binary handles multiple sensors:

```
                       ┌─────────────────────┐
/imu/data ────────────►│  frame_validator     │──► /sensor/frame_status
                       │  (expected_frame_id  │
                       │   param: "imu_link") │
                       └─────────────────────┘
```

### Pattern B — Multi-sensor health monitor

```
/imu/data          ──┐
/odom              ──┼──► [Seen structs] ──► [Timer 500ms] ──► /sensor/health
/front_laser/scan  ──┘    (last seen time)    (stale check)
```

### Pattern C — Rate estimator

Track sample count over a sliding window to compute actual Hz:

```cpp
struct RateEstimator {
    rclcpp::Time window_start;
    int count{0};
    double hz{0.0};

    void mark(const rclcpp::Time& now) {
        ++count;
        double elapsed = (now - window_start).seconds();
        if (elapsed > 1.0) {
            hz = count / elapsed;
            count = 0;
            window_start = now;
        }
    }
};
```
