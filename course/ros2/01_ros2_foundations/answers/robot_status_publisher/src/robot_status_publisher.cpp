#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

class RobotStatusPublisher : public rclcpp::Node {
public:
    RobotStatusPublisher()
        : rclcpp::Node("robot_status_publisher")
    {
        const double rate_hz = declare_parameter<double>("rate_hz", 1.0);
        if (rate_hz <= 0.0 || !std::isfinite(rate_hz)) {
            throw std::runtime_error("rate_hz must be positive and finite");
        }

        publisher_ = create_publisher<std_msgs::msg::String>("/robot/status_text", 10);

        reset_service_ = create_service<std_srvs::srv::Trigger>(
            "reset_counter",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                counter_ = 0;
                response->success = true;
                response->message = "counter reset";
            });

        const auto period = std::chrono::duration<double>(1.0 / rate_hz);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this] { publish_status(); });

        RCLCPP_INFO(get_logger(), "publishing /robot/status_text at %.3f Hz", rate_hz);
    }

private:
    void publish_status() {
        std_msgs::msg::String msg;
        msg.data = "status count=" + std::to_string(counter_++);
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    std::uint64_t counter_{0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotStatusPublisher>());
    rclcpp::shutdown();
    return 0;
}

