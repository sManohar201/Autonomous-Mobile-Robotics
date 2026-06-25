#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;
struct Cache { std::optional<std::string> scan, imu, odom; std::mutex mutex; };
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("concurrent_pipeline_hard");
    auto cache = std::make_shared<Cache>();
    auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions opts; opts.callback_group = group;
    auto make_sub = [&](const std::string& topic, auto setter) {
        return node->create_subscription<std_msgs::msg::String>(topic, 10, [cache, setter](const std_msgs::msg::String::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(cache->mutex); setter(*cache, msg->data);
        }, opts);
    };
    auto scan = make_sub("/scan_status", [](Cache& c, const std::string& v) { c.scan = v; });
    auto imu = make_sub("/imu_status", [](Cache& c, const std::string& v) { c.imu = v; });
    auto odom = make_sub("/odom_status", [](Cache& c, const std::string& v) { c.odom = v; });
    auto pub = node->create_publisher<std_msgs::msg::String>("/pipeline/summary", 10);
    auto timer = node->create_wall_timer(200ms, [cache, pub] {
        Cache snapshot;
        { std::lock_guard<std::mutex> lock(cache->mutex); snapshot.scan = cache->scan; snapshot.imu = cache->imu; snapshot.odom = cache->odom; }
        std_msgs::msg::String msg; msg.data = (snapshot.scan && snapshot.imu && snapshot.odom) ? "OK" : "MISSING_INPUTS"; pub->publish(msg);
    });
    (void)scan; (void)imu; (void)odom;
    rclcpp::spin(node); rclcpp::shutdown(); return 0;
}
