#include <chrono>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RobotMonitorPrototype : public rclcpp::Node {
public:
    RobotMonitorPrototype()
        : rclcpp::Node("robot_monitor_prototype")
    {
        const auto input_topic = declare_parameter<std::string>("input_topic", "/robot/status_text");
        const int stale_timeout_ms = declare_parameter<int>("stale_timeout_ms", 1000);
        const int check_period_ms = declare_parameter<int>("check_period_ms", 200);

        if (input_topic.empty()) {
            throw std::runtime_error("input_topic must be non-empty");
        }
        if (stale_timeout_ms <= 0) {
            throw std::runtime_error("stale_timeout_ms must be positive");
        }
        if (check_period_ms <= 0) {
            throw std::runtime_error("check_period_ms must be positive");
        }

        stale_timeout_ = rclcpp::Duration::from_nanoseconds(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::milliseconds(stale_timeout_ms)).count());

        summary_pub_ = create_publisher<std_msgs::msg::String>("/robot/monitor_summary", 10);
        status_sub_ = create_subscription<std_msgs::msg::String>(
            input_topic,
            10,
            [this](const std_msgs::msg::String& msg) { on_status(msg); });

        timer_ = create_wall_timer(
            std::chrono::milliseconds(check_period_ms),
            [this] { check_health(); });

        RCLCPP_INFO(get_logger(), "monitoring %s with stale timeout %d ms",
                    input_topic.c_str(), stale_timeout_ms);
    }

private:
    void on_status(const std_msgs::msg::String& msg) {
        last_message_ = msg.data;
        last_seen_ = now();
        seen_any_ = true;
    }

    void check_health() {
        std_msgs::msg::String summary;
        if (!seen_any_) {
            summary.data = "STALE no messages received";
        } else if ((now() - last_seen_) > stale_timeout_) {
            summary.data = "STALE last='" + last_message_ + "'";
        } else {
            summary.data = "OK last='" + last_message_ + "'";
        }
        summary_pub_->publish(summary);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_seen_;
    rclcpp::Duration stale_timeout_{0, 0};
    std::string last_message_;
    bool seen_any_{false};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotMonitorPrototype>());
    rclcpp::shutdown();
    return 0;
}

