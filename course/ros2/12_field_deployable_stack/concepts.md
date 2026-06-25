# ROS2 Module 12 — Field Deployable Stack

## Goal

Assemble a field-deployable robot stack: startup sequencing, configuration layers,
deployment readiness checks, degraded modes, recovery, and operator workflows.
This module ties together all prior modules into a production architecture.

---

## 1. Deployment Architecture Overview

A field-deployable ROS2 stack has distinct layers:

```
┌─────────────────────────────────────────────────────────┐
│  Operator interface                                       │
│    /deployment/readiness   /operator/health              │
│    /operator/fault_report  /robot/degraded_mode          │
├─────────────────────────────────────────────────────────┤
│  Stack orchestration                                      │
│    deployment_checklist_node  degraded_mode_manager      │
│    diagnostic_aggregator      fault_report_hard          │
├─────────────────────────────────────────────────────────┤
│  Subsystems                                               │
│    localization  planning  control  sensor_health        │
│    each publishes → /subsystem/status                    │
├─────────────────────────────────────────────────────────┤
│  Drivers / Simulation                                     │
│    hardware drivers  OR  ros_gz_bridge + Gazebo          │
├─────────────────────────────────────────────────────────┤
│  Configuration                                            │
│    config/robots/<name>.yaml  config/envs/<env>.yaml    │
└─────────────────────────────────────────────────────────┘
```

---

## 2. Deployment Readiness Checklist

Before a robot can safely start a mission, a set of preconditions must be
confirmed. The `deployment_checklist_node` encodes these as boolean parameters
and publishes a `ready=true/false` verdict:

```cpp
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("deployment_checklist_node");

    node->declare_parameter<bool>("config_loaded",       false);
    node->declare_parameter<bool>("diagnostics_ready",   false);
    node->declare_parameter<bool>("bagging_ready",       false);
    node->declare_parameter<bool>("operator_confirmed",  false);

    auto pub = node->create_publisher<std_msgs::msg::String>(
        "deployment/readiness", 10);

    auto timer = node->create_wall_timer(std::chrono::seconds(1),
        [node, pub] {
            const bool config     = node->get_parameter("config_loaded").as_bool();
            const bool diag       = node->get_parameter("diagnostics_ready").as_bool();
            const bool bag        = node->get_parameter("bagging_ready").as_bool();
            const bool confirmed  = node->get_parameter("operator_confirmed").as_bool();
            const bool ready      = config && diag && bag && confirmed;

            std::ostringstream s;
            s << "ready=" << (ready ? "true" : "false")
              << " config_loaded="      << config
              << " diagnostics_ready="  << diag
              << " bagging_ready="      << bag
              << " operator_confirmed=" << confirmed;
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

### Completing checklist items at runtime

```bash
# Mark config loaded after verifying parameters are correct
ros2 param set /deployment_checklist_node config_loaded true

# Mark operator confirmed after visual inspection
ros2 param set /deployment_checklist_node operator_confirmed true

# Watch readiness
ros2 topic echo /deployment/readiness
```

---

## 3. Degraded Modes

A field robot must define a safe policy for every fault combination. Not every
fault requires full shutdown — but every fault needs a documented response.

### Mode table

| Fault condition | Safe mode | Rationale |
|---|---|---|
| `control:FAILED` OR `safety:FAILED` | `STOP` | Cannot actuate safely |
| `localization:FAILED` | `LOCALIZE_ONLY` | Can perceive but cannot navigate |
| `sensor:DEGRADED` OR `planning:DEGRADED` | `SLOW` | Reduced confidence → reduced speed |
| All OK | `NORMAL` | Full autonomy |

### Degraded mode selector

```cpp
std::string select_mode(const std::string& health) {
    if (health.find("control:FAILED")   != std::string::npos ||
        health.find("safety:FAILED")    != std::string::npos) {
        return "STOP";
    }
    if (health.find("localization:FAILED") != std::string::npos) {
        return "LOCALIZE_ONLY";
    }
    if (health.find("sensor:DEGRADED")   != std::string::npos ||
        health.find("planning:DEGRADED") != std::string::npos) {
        return "SLOW";
    }
    return "NORMAL";
}
```

```cpp
// Subscribe to aggregated health, publish selected mode every 500 ms
auto sub = node->create_subscription<std_msgs::msg::String>(
    "subsystem/health", 10,
    [health](const std_msgs::msg::String::SharedPtr msg) { *health = msg->data; });

auto timer = node->create_wall_timer(500ms, [output, health] {
    std_msgs::msg::String msg;
    msg.data = select_mode(*health);
    output->publish(msg);
});
```

### Data flow

```
/subsystem/status (multiple publishers)
        │
        ▼
diagnostic_aggregator
        │
        ▼ /subsystem/health "robot_health=DEGRADED sensor:DEGRADED:..."
        │
        ▼
degraded_mode_manager
        │
        ▼ /robot/degraded_mode "SLOW"
        │
        ▼
navigation stack (reads mode, adjusts max velocity / replanning rate)
```

---

## 4. Recovery Behavior Principles

Recovery must be bounded and observable. Unbounded silent retries are dangerous.

### Rules for recovery logic

1. **Count attempts** — after N failures, escalate rather than retry
2. **Log each attempt** — operators must see what the system tried
3. **Publish state** — recovery state goes to `/robot/degraded_mode` so it's visible
4. **Respect a timeout** — if not recovered within T seconds, stop and alert operator
5. **Preserve evidence** — trigger a bag record at the moment of fault

```cpp
// Recovery attempt with bounded retry count
int recovery_attempts = 0;
constexpr int max_recovery_attempts = 3;

// In fault handling:
if (recovery_attempts < max_recovery_attempts) {
    RCLCPP_WARN(node->get_logger(), "Recovery attempt %d of %d",
                ++recovery_attempts, max_recovery_attempts);
    // trigger recovery action
} else {
    RCLCPP_ERROR(node->get_logger(),
                 "Recovery exhausted after %d attempts — escalating to STOP",
                 max_recovery_attempts);
    // publish STOP mode, alert operator
}
```

---

## 5. Stack Launch Hierarchy

A production stack has three launch levels:

```
field.launch.py
    ├── drivers.launch.py      (hardware or bridge)
    ├── localization.launch.py
    ├── planning.launch.py
    ├── control.launch.py
    └── monitoring.launch.py
            ├── deployment_checklist_node
            ├── diagnostic_aggregator
            ├── fault_report_hard
            └── degraded_mode_manager
```

Each layer is independently launchable for subsystem testing. The top-level
`field.launch.py` assembles the full stack:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('my_robot')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'monitoring.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'localization.launch.py'))),
    ])
```

---

## 6. Configuration at the Stack Level

Stack-level YAML ties robot identity to environment:

```yaml
# config/robots/pallet_jack.yaml
deployment_checklist_node:
  ros__parameters:
    config_loaded: false
    operator_confirmed: false

degraded_mode_manager:
  ros__parameters: {}

# config/envs/field.yaml
/**:
  ros__parameters:
    use_sim_time: false
    config_profile: field
```

The `/**:` wildcard applies parameters to ALL nodes in the stack — useful for
`use_sim_time` and environment identity.

---

## 7. Logging and Bagging Policy

### Pre-mission

```bash
# Start recording key topics before mission
ros2 bag record \
  /odom /imu/data /front_laser/scan \
  /localization/status /subsystem/status /operator/health \
  /robot/degraded_mode /deployment/readiness \
  -o mission_$(date +%Y%m%d_%H%M%S)
```

### Incident-triggered recording

```python
# In a monitoring node: start bag on fault
if mode == "STOP" and not bag_running:
    subprocess.Popen(['ros2', 'bag', 'record', '-a', '-o', 'incident_bag'])
    bag_running = True
```

### Retention policy

| Data | Retention | Reason |
|---|---|---|
| Normal mission bags | 7 days | Storage cost vs. debugging value |
| Incident bags | 90 days | Required for post-mortem |
| CI regression bags | Indefinite | Test fixtures — never delete |

---

## 8. Operator Runbooks

For each fault mode, document the runbook in a human-readable format:

**STOP mode:**
1. Robot has halted. Do not attempt to drive.
2. Check `/operator/fault_report` for reason.
3. If `control:FAILED`: inspect CAN bus and motor driver logs.
4. Download incident bag for engineering review.
5. Contact on-call engineer before restarting.

**LOCALIZE_ONLY mode:**
1. Robot can perceive but not navigate autonomously.
2. Check `/localization/status` for `accepted` count and `update_age_s`.
3. If `update_age_s > 5`: check odometry topic with `ros2 topic hz /odom`.
4. Reset localization: `ros2 service call /localization/reset std_srvs/srv/Trigger`
5. If reset fails to recover within 60 s, escalate to STOP.

---

## 9. Full Stack Readiness Checklist

Before a production deployment:

**Software:**
- [ ] All packages built from a tagged release — not `main` branch HEAD
- [ ] All answers packages compile with zero warnings (`-Wall -Wextra`)
- [ ] CI passed on the release tag (build + unit + launch + lint)
- [ ] Simulation test bag replay passed for this release

**Configuration:**
- [ ] Robot-specific YAML loaded and verified (`ros2 param dump --all`)
- [ ] `use_sim_time: false` for all nodes in field launch
- [ ] Bag recording confirmed active before mission start

**Runtime:**
- [ ] `/deployment/readiness` shows `ready=true`
- [ ] `/operator/health` shows `robot_health=OK`
- [ ] `/robot/degraded_mode` shows `NORMAL`
- [ ] `ros2 topic hz` on all sensor topics shows expected rates

---

## 10. Production Trade-offs

| Decision | Trade-off |
|---|---|
| Readiness as parameters vs external signals | Parameters: simple. External: auto-confirms when system comes up |
| Mode as string vs enum | String: human-readable in logs. Enum: type-safe, no typo risk |
| Degraded mode published at 2 Hz | Low enough for nav stack to poll. High enough for operator awareness |
| Single aggregator topic vs per-subsystem | Single: one subscription for everything. Per-subsystem: finer-grained routing |
| Bounded recovery vs infinite retry | Bounded: operators informed quickly. Infinite: may self-heal but hides problems |

---

## 11. Summary — What a Field-Deployable Stack Looks Like

```
Startup:  driver launch → bridge → robot_state_publisher → navigation stack
Config:   robot YAML + env YAML + operator overrides
Monitor:  each subsystem → /subsystem/status → aggregator → /operator/health
Degrade:  aggregator → /subsystem/health → degraded_mode_manager → /robot/degraded_mode
Recover:  bounded retries + RCLCPP_WARN per attempt + bag on fault
Evidence: continuous bag (ring buffer) + incident bag on STOP/FAILED
Runbooks: documented per fault mode, match what the code actually does
```

A stack is field-deployable when operators can read its health, understand
its faults, act on its runbooks, and engineers can reproduce any incident
from a bag.
