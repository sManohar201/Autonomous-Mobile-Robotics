# 4. Predict and Update Lifecycle

State estimators have two phases that should be called separately:

```
Time t0: odom callback arrives
   │
   ├─ predict(stamp=t0)   ← propagates state forward to t0 using motion model
   └─ update(measurement) ← corrects state with sensor reading
```

Even for a simple filter shell, keeping them separate preserves the interface
contract that a real filter (EKF, UKF) will use:

```cpp
void predict(const rclcpp::Time& stamp) {
    // Real EKF: propagate state using dynamics (ẋ = Ax + Bu)
    // Shell: just track last prediction time
    last_predict_stamp_ = stamp;
}
```

```cpp
// In subscription callback:
estimator->predict(msg->header.stamp);
std::string reason;
const bool ok = estimator->update(*msg, reason);
```
