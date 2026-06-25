#include <chrono>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

static bool forbidden_edge(const std::string& from, const std::string& to) {
    if (from.find("interfaces") != std::string::npos && to.find("monitoring") != std::string::npos) return true;
    if (from.find("description") != std::string::npos && to.find("navigation") != std::string::npos) return true;
    return false;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("dependency_audit_hard");
    auto pub = node->create_publisher<std_msgs::msg::String>("/architecture/dependency_audit", 10);

    const std::map<std::string, std::vector<std::string>> deps{
        {"robot_interfaces", {}},
        {"robot_monitoring", {"robot_interfaces"}},
        {"robot_bringup", {"robot_interfaces", "robot_monitoring"}},
        {"robot_description", {}},
    };

    auto timer = node->create_wall_timer(1s, [pub, deps] {
        std::ostringstream report;
        bool ok = true;
        for (const auto& [from, tos] : deps) {
            for (const auto& to : tos) {
                if (forbidden_edge(from, to)) {
                    ok = false;
                    report << "forbidden " << from << "->" << to << " ";
                }
            }
        }
        std_msgs::msg::String msg;
        msg.data = ok ? "dependency audit OK" : report.str();
        pub->publish(msg);
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

