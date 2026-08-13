# 3. Severity Ranking and Aggregation

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
