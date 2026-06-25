# ROS2 Module 07 — State Estimation Architecture

## Goal

Design a localization subsystem with clean ROS2 interfaces: measurement adapters
that decouple filter logic from message types, rejection logic (wrong frame, high
covariance, outlier jumps), a reset service, and diagnostics. The filter core
should be testable in isolation without a running ROS2 system.

> **See also:** https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html
> for executor patterns used to spin the state estimation node. Module 06 covers
> sensor topic QoS that feeds this subsystem.

---

## 1. Why "Architecture" Not Just "Filter"

A deployable localization system has layers:

```
                     ROS2 layer
   ┌────────────────────────────────────────────────────┐
   │  /odom subscription → MeasurementAdapter            │
   │  /localization/reset service                        │
   │  /localization/status publisher (diagnostics timer) │
   └──────────────────────────┬─────────────────────────┘
                              │ typed measurement struct
                     Filter layer (pure C++)
   ┌──────────────────────────────────────────────────────┐
   │  StateEstimator                                       │
   │    predict(stamp)                                     │
   │    update(measurement, &reason) → accepted/rejected   │
   │    reset()                                            │
   │    diagnostics(clock) → string                        │
   └──────────────────────────────────────────────────────┘
```

The separation exists because:
- **Testability:** The filter can be unit-tested with deterministic inputs, no
  DDS stack needed.
- **Portability:** The filter core can run in a simulation harness, on hardware,
  or in offline analysis tools.
- **Clarity:** Every ROS2 integration concern (frame IDs, timestamps, QoS)
  lives in the adapter; the filter sees clean structs.

---

## 2. Measurement Adapters

A measurement adapter converts a ROS2 message into a plain C++ struct that the
filter understands. The adapter also extracts the information the filter needs
(e.g., yaw from quaternion).

### Why extract yaw from the quaternion?

`nav_msgs/Odometry` carries orientation as a quaternion `[x, y, z, w]`. A 2D
state estimator only needs yaw. Extracting it in the adapter keeps the filter
core free of quaternion math:

```cpp
struct PoseMeasurement {
    std::string frame_id;
    double x{};
    double y{};
    double yaw{};
    double covariance_trace{};
};

PoseMeasurement adapt_odometry(const nav_msgs::msg::Odometry& msg) {
    const auto& q = msg.pose.pose.orientation;

    // Yaw from quaternion: atan2(2(wz + xy), 1 - 2(yy + zz))
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

    PoseMeasurement m;
    m.frame_id = msg.header.frame_id;
    m.x = msg.pose.pose.position.x;
    m.y = msg.pose.pose.position.y;
    m.yaw = std::atan2(siny_cosp, cosy_cosp);
    // Trace of x, y, yaw covariance sub-block
    m.covariance_trace = msg.pose.covariance[0]   // var(x)
                       + msg.pose.covariance[7]   // var(y)
                       + msg.pose.covariance[35]; // var(yaw)
    return m;
}
```

The covariance trace (sum of diagonal elements) gives a single scalar for
"how uncertain is this measurement overall."

### Using the adapter

```cpp
auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    [output](const nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto m = adapt_odometry(*msg);

        std::ostringstream line;
        line << "frame=" << m.frame_id
             << " x=" << m.x << " y=" << m.y
             << " yaw=" << m.yaw
             << " cov_trace=" << m.covariance_trace;

        std_msgs::msg::String out;
        out.data = line.str();
        output->publish(out);
    });
```

---

## 3. Measurement Rejection

Filters should reject bad measurements rather than letting them corrupt state.
Three common rejection conditions:

| Condition | Check | Reason |
|---|---|---|
| Wrong frame | `frame_id != required_frame` | Data in wrong coordinate frame |
| High covariance | `cov_trace > threshold` | Sensor reporting low confidence |
| Outlier jump | `hypot(dx, dy) > max_jump` | Position jumped implausibly far |

```cpp
bool update(const nav_msgs::msg::Odometry& msg, std::string& reason) {
    if (msg.header.frame_id != required_frame_) {
        reason = "wrong_frame";
        ++rejected_;
        return false;
    }

    const double cov_trace = msg.pose.covariance[0]
                           + msg.pose.covariance[7]
                           + msg.pose.covariance[35];
    if (cov_trace > max_cov_trace_) {
        reason = "high_covariance";
        ++rejected_;
        return false;
    }

    const double dx = msg.pose.pose.position.x - x_;
    const double dy = msg.pose.pose.position.y - y_;
    if (initialized_ && std::hypot(dx, dy) > max_jump_m_) {
        reason = "outlier_jump";
        ++rejected_;
        return false;
    }

    // Accepted — update state
    x_ = msg.pose.pose.position.x;
    y_ = msg.pose.pose.position.y;
    initialized_ = true;
    ++accepted_;
    reason = "accepted";
    return true;
}
```

Log rejections with throttle to avoid log spam at 50 Hz:

```cpp
if (!accepted) {
    RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000,  // max 1 warning per 2 s
        "Rejected measurement: %s", reason.c_str());
}
```

> **Interview question:** "Your EKF keeps rejecting GPS updates. What's your
> investigation order?"
> 1. Check `frame_id` — wrong frame? Missing TF transform?
> 2. Check `covariance_trace` — sensor reporting very high uncertainty?
> 3. Check `outlier_jump` threshold — is the jump threshold too tight?
> 4. Look at the actual pose values — is GPS in a different datum or projection?

---

## 4. Predict and Update Lifecycle

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

---

## 5. Reset Service

A `std_srvs/Trigger` service is the standard interface for resetting a ROS2
subsystem. It takes no parameters and returns `{success, message}`.

```cpp
#include "std_srvs/srv/trigger.hpp"

auto reset_srv = node->create_service<std_srvs::srv::Trigger>(
    "localization/reset",
    [estimator](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        estimator->reset();
        resp->success = true;
        resp->message = "estimator reset";
    });
```

Call from the command line:
```bash
ros2 service call /localization/reset std_srvs/srv/Trigger
```

### What reset() should do

```cpp
void reset() {
    x_ = 0.0;
    y_ = 0.0;
    accepted_ = 0;
    rejected_ = 0;
    initialized_ = false;
    // Real filter: also reset covariance to initial P0
}
```

> **Production consideration:** Should you allow reset while the robot is moving?
> Some robots reject reset requests if velocity exceeds a threshold — the filter
> would immediately receive a large jump from the current motion state.
> Extend the service to check velocity and return `success=false` with a
> descriptive message if unsafe.

---

## 6. Diagnostics

Diagnostics tell operators what the estimator is doing: how many measurements
were accepted/rejected, whether the filter is initialised, and when it last
received a useful update.

```cpp
std::string diagnostics(const rclcpp::Clock& clock) const {
    std::ostringstream out;
    out << "initialized=" << (initialized_ ? "true" : "false")
        << " accepted=" << accepted_
        << " rejected=" << rejected_
        << " state=(" << x_ << "," << y_ << ")"
        << " predict_age_s=" << (clock.now() - last_predict_stamp_).seconds()
        << " update_age_s="  << (clock.now() - last_update_stamp_).seconds();
    return out.str();
}
```

Publish on a timer so monitoring tools can read it independently:

```cpp
auto diag_pub = node->create_publisher<std_msgs::msg::String>(
    "localization/status", 10);

auto diag_timer = node->create_wall_timer(500ms, [estimator, diag_pub, node] {
    std_msgs::msg::String msg;
    msg.data = estimator->diagnostics(*node->get_clock());
    diag_pub->publish(msg);
});
```

---

## 7. Full StateEstimator Class

```cpp
class StateEstimator {
public:
    explicit StateEstimator(std::string required_frame)
        : required_frame_(std::move(required_frame)) {}

    void predict(const rclcpp::Time& stamp) {
        last_predict_stamp_ = stamp;
    }

    bool update(const nav_msgs::msg::Odometry& msg, std::string& reason) {
        if (msg.header.frame_id != required_frame_) {
            reason = "wrong_frame"; ++rejected_; return false;
        }
        const double cov = msg.pose.covariance[0]
                         + msg.pose.covariance[7]
                         + msg.pose.covariance[35];
        if (cov > max_cov_trace_) {
            reason = "high_covariance"; ++rejected_; return false;
        }
        const double dx = msg.pose.pose.position.x - x_;
        const double dy = msg.pose.pose.position.y - y_;
        if (initialized_ && std::hypot(dx, dy) > max_jump_m_) {
            reason = "outlier_jump"; ++rejected_; return false;
        }
        x_ = msg.pose.pose.position.x;
        y_ = msg.pose.pose.position.y;
        last_update_stamp_ = msg.header.stamp;
        initialized_ = true;
        ++accepted_;
        reason = "accepted";
        return true;
    }

    void reset() {
        x_ = 0.0; y_ = 0.0;
        accepted_ = 0; rejected_ = 0;
        initialized_ = false;
    }

    std::string diagnostics(const rclcpp::Clock& clock) const {
        std::ostringstream out;
        out << "initialized=" << (initialized_ ? "true" : "false")
            << " accepted=" << accepted_
            << " rejected=" << rejected_
            << " predict_age_s=" << (clock.now() - last_predict_stamp_).seconds()
            << " update_age_s="  << (clock.now() - last_update_stamp_).seconds();
        return out.str();
    }

private:
    std::string required_frame_;
    double max_cov_trace_{1.0};
    double max_jump_m_{3.0};
    double x_{0.0}, y_{0.0};
    int accepted_{0}, rejected_{0};
    bool initialized_{false};
    rclcpp::Time last_predict_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_update_stamp_{0, 0, RCL_ROS_TIME};
};
```

---

## 8. Architecture Diagram

```
/odom topic
    │ nav_msgs/Odometry
    ▼
[adapt_odometry()]  ←── extract x, y, yaw, cov_trace
    │ PoseMeasurement
    ▼
[StateEstimator::predict(stamp)]
[StateEstimator::update(msg, &reason)]
    │ accepted / rejected
    ├──► RCLCPP_WARN_THROTTLE (on rejection)
    │
    ├──► [Timer 500ms]
    │         └──► /localization/status (diagnostics)
    │
    └──► [Reset service]
              └── /localization/reset → estimator.reset()
```

---

## 9. Production Trade-offs

| Decision | Trade-off |
|---|---|
| Adapter vs direct filter access | Adapter: decoupled, testable. Direct: less code, harder to test |
| Rejection vs clipping | Rejection: clean, debug-visible via reason string. Clipping: filter keeps running, silently less accurate |
| `Trigger` vs custom service | Trigger: standard, no schema to define. Custom: carries parameters (reset pose, covariance) but adds complexity |
| Diagnostics rate | 0.5–2 Hz is typical — operators need it readable, not performance-critical |
| `initialized_` flag | Guards against large `outlier_jump` rejections before any update — correct filter behaviour |
| Covariance trace threshold | Too tight: rejects valid measurements in noisy environments. Too loose: accepts bad measurements. Tune per sensor spec. |

---

## 10. Testing Without Hardware

Use `ros2 topic pub` to inject synthetic odometry:

```bash
# Inject a valid odom message
ros2 topic pub /odom nav_msgs/msg/Odometry \
  '{header: {frame_id: odom}, pose: {pose: {position: {x: 1.0, y: 2.0}}}}' \
  --once

# Inject wrong frame to test rejection
ros2 topic pub /odom nav_msgs/msg/Odometry \
  '{header: {frame_id: wrong_frame}}' \
  --once

# Check diagnostics
ros2 topic echo /localization/status --once

# Reset
ros2 service call /localization/reset std_srvs/srv/Trigger
```

---

## 11. Checklist

- [ ] Adapter is a pure function — no global state, no ROS calls
- [ ] All three rejection conditions are implemented and logged
- [ ] Reset clears state AND counters (accepted, rejected, initialized)
- [ ] Diagnostics include `update_age_s` — detects silent measurement loss
- [ ] Reset service returns meaningful `message` field for all outcomes
- [ ] Filter core (StateEstimator) has no `#include "rclcpp/rclcpp.hpp"` — keeps it testable in isolation
