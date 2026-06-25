# ROS2 Module 11 — Observability & Operations

## Goal

Design robot software so field operators and engineers can understand what the
robot is doing and why it failed. Observability is a first-class requirement —
a robot that fails silently is more dangerous than one that reports clearly.

> **See also:** https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Logging.html
> for the ROS2 logging API. This module goes beyond logging to cover the full
> observability stack: diagnostics, aggregation, fault reports, and operational
> workflows.

---

## 1. Observability Signals

Production robot systems need five types of observability:

| Signal | Form | Who reads it | Tool |
|---|---|---|---|
| **Diagnostics** | Structured health status per subsystem | Operators, dashboards | `/diagnostic_agg`, custom topics |
| **Logs** | Timestamped text per severity | Engineers debugging | `rcl_logging`, `ros2 topic echo /rosout` |
| **Bags** | Full topic recordings | Post-mortem analysis | `ros2 bag record` |
| **Metrics** | Numerical counters / rates | Monitoring dashboards | Custom publishers |
| **Fault reports** | Structured fault + action | Operators in the field | Custom topic |

---

## 2. Diagnostic Health Levels

Use exactly three levels, consistently:

| Level | Meaning | Examples |
|---|---|---|
| `OK` | All checks pass | Sensor publishing at expected rate, frame IDs correct |
| `DEGRADED` | Reduced capability, still safe to operate | One of three sensors stale, navigation slower |
| `FAILED` | Cannot fulfil primary function | All localization measurements rejected |

The key principle: **a subsystem declares its own health, an aggregator collects
and rolls up to robot-level health**. Never hard-code the overall health string in
the monitoring node.

### Status string format

```
subsystem_name:status:detail
localization:DEGRADED:rejected=12 accepted=0 since=3.2s
```

The `:` separators make it easy to parse without a schema:
- `find(':')` extracts the subsystem name
- `severity_rank()` maps status text to an integer for rolling up

---

## 3. Severity Ranking and Aggregation

An aggregator subscribes to multiple `subsystem/status` topics and rolls them
up to a single `robot_health` verdict:

```cpp
int severity_rank(const std::string& status) {
    if (status.find("FAILED")   != std::string::npos) return 2;
    if (status.find("DEGRADED") != std::string::npos ||
        status.find("STALE")    != std::string::npos) return 1;
    return 0;
}

const char* label_for(int rank) {
    if (rank >= 2) return "FAILED";
    if (rank == 1) return "DEGRADED";
    return "OK";
}
```

The aggregation rule: overall health is the worst of all subsystem healths.

### Full diagnostic aggregator

```cpp
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

int severity_rank(const std::string& s) {
    if (s.find("FAILED")   != std::string::npos) return 2;
    if (s.find("DEGRADED") != std::string::npos ||
        s.find("STALE")    != std::string::npos) return 1;
    return 0;
}
const char* label_for(int r) { return r>=2?"FAILED":r==1?"DEGRADED":"OK"; }

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node     = std::make_shared<rclcpp::Node>("diagnostic_aggregator");
    auto statuses = std::make_shared<std::map<std::string, std::string>>();

    // Single subscription — each subsystem prefixes its message with "name:"
    auto input = node->create_subscription<std_msgs::msg::String>(
        "subsystem/status", 10,
        [statuses](const std_msgs::msg::String::SharedPtr msg) {
            const auto split = msg->data.find(':');
            const auto name  = split == std::string::npos
                               ? "unknown" : msg->data.substr(0, split);
            (*statuses)[name] = msg->data;
        });

    auto output = node->create_publisher<std_msgs::msg::String>(
        "operator/health", 10);

    auto timer = node->create_wall_timer(500ms, [output, statuses] {
        int worst = 0;
        std::ostringstream out;
        out << "subsystems=" << statuses->size() << " ";
        for (const auto& [name, status] : *statuses) {
            worst = std::max(worst, severity_rank(status));
            out << name << "={" << status << "} ";
        }
        std_msgs::msg::String msg;
        msg.data = std::string("robot_health=") + label_for(worst) + " " + out.str();
        output->publish(msg);
    });

    (void)input; (void)timer;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Topic wiring for aggregation

```
localization node      ──► /subsystem/status "localization:OK:..."
sensor_health node     ──► /subsystem/status "sensors:DEGRADED:..."
planning node          ──► /subsystem/status "planning:OK:..."
                                │
                    diagnostic_aggregator
                                │
                       /operator/health "robot_health=DEGRADED subsystems=3 ..."
```

All subsystems publish to the same topic with their name as prefix. The
aggregator uses a `std::map<name, status>` to track the latest from each.

---

## 4. Fault Reports

A fault report is a structured message that tells an operator:
- **What** failed (subsystem name)
- **How bad** it is (severity)
- **Why** it failed (reason)
- **What to do** (action)

```cpp
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("fault_report_hard");

    node->declare_parameter<std::string>("subsystem", "localization");
    node->declare_parameter<std::string>("severity",  "DEGRADED");
    node->declare_parameter<std::string>("reason",    "stale odometry");
    node->declare_parameter<std::string>("action",    "verify odom publisher and inspect latest bag");

    auto pub = node->create_publisher<std_msgs::msg::String>(
        "operator/fault_report", 10);

    auto timer = node->create_wall_timer(std::chrono::seconds(1),
        [node, pub] {
            std::ostringstream s;
            s << "stamp_ns=" << node->now().nanoseconds()
              << " subsystem=" << node->get_parameter("subsystem").as_string()
              << " severity="  << node->get_parameter("severity").as_string()
              << " reason=\""  << node->get_parameter("reason").as_string()  << "\""
              << " action=\""  << node->get_parameter("action").as_string()  << "\"";
            std_msgs::msg::String msg;
            msg.data = s.str();
            pub->publish(msg);
        });

    (void)timer;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

Making these values parameters means the same binary can report any fault — the
launch file or operator inject the specifics at runtime.

---

## 5. Logging Best Practices

### Severity levels

```cpp
RCLCPP_DEBUG(logger, "Processing scan: %zu points", cloud.size());
RCLCPP_INFO(logger, "Localization initialized at (%.2f, %.2f)", x_, y_);
RCLCPP_WARN(logger, "Rejected measurement: %s", reason.c_str());
RCLCPP_ERROR(logger, "State estimator failed: no valid measurements in %.1f s", age);
RCLCPP_FATAL(logger, "Unsupported configuration: %s", config.c_str());
```

| Level | When to use |
|---|---|
| `DEBUG` | Internal state, high-frequency (rate-limited!) |
| `INFO` | State transitions, lifecycle events |
| `WARN` | Recoverable anomalies — operator should know |
| `ERROR` | Failed an operation — subsystem degraded |
| `FATAL` | Unrecoverable — node will exit |

### Rate limiting logs

```cpp
// At most one warning per 2 s — critical for 50 Hz sensor callbacks
RCLCPP_WARN_THROTTLE(
    node->get_logger(), *node->get_clock(), 2000,
    "Rejecting measurement: %s", reason.c_str());

// Log only on first occurrence
RCLCPP_WARN_ONCE(node->get_logger(), "First stale input detected");
```

Without throttling, a 50 Hz callback that logs every time it rejects fills
disk in minutes and buries useful signals in noise.

---

## 6. Bag Recording for Operations

A bag captures the full topic graph at a point in time. It is the primary
evidence preservation tool for post-mortem analysis.

```bash
# Record all topics to named bag
ros2 bag record -a -o fault_2026_06_25

# Record only the topics needed for localization post-mortem
ros2 bag record /odom /imu/data /front_laser/scan /localization/status \
  -o localization_incident

# Replay
ros2 bag play localization_incident

# Inspect
ros2 bag info localization_incident
```

### Operational bag strategy

| Use case | Approach |
|---|---|
| Continuous ring buffer | `--storage-preset sqlite3` + auto-split by size |
| Incident capture | Trigger bag record on fault event; stop after 60 s |
| Test fixtures | Record known-good runs; replay in CI for regression testing |

---

## 7. Operational Workflows

Every fault class should have a documented workflow:

```
Fault: DEGRADED (stale sensor)
  1. Detect: subsystem/status shows STALE_INPUTS
  2. Report: fault_report published with reason + action
  3. Degrade: localization enters lower-confidence mode (wider covariance)
  4. Recover: if sensor comes back within 5 s, auto-resume
  5. Evidence: bag of last 60 s automatically preserved
  6. Stop: if no recovery after 30 s, halt navigation, wait for operator
```

This workflow is a contract — the software must implement each step and the
operator's runbook must match.

---

## 8. Production Trade-offs

| Decision | Trade-off |
|---|---|
| Single aggregation topic vs multiple | Single `/subsystem/status`: one subscription. Multiple: cleaner per-subsystem routing |
| Text severity vs numeric | Text (`"DEGRADED"`): readable in logs. Numeric: easier to compare in code |
| `RCLCPP_WARN` vs publish to `/faults` | Warn: ephemeral, lost without a log subscriber. Topic: persistent, queryable |
| Continuous bag recording | High storage cost. Use ring-buffer with auto-overwrite or incident-triggered recording |
| Fault with action vs without | With action: actionable for operators. Without: still useful for engineers |

---

## 9. Checklist

- [ ] Every subsystem publishes to `subsystem/status` with its name as prefix
- [ ] Severity uses exactly `OK`, `DEGRADED`, `FAILED` — not free-form text
- [ ] All warning-level logs in high-frequency callbacks use `RCLCPP_WARN_THROTTLE`
- [ ] Fault reports include `reason` + `action` — operators can act without reading source code
- [ ] Bag recording strategy is documented: what to record and for how long
- [ ] Aggregator outputs worst-of-all-subsystems health at operator-visible topic
- [ ] Health topic published at a rate operators can read (0.5–2 Hz) not sensor rate (50–100 Hz)
