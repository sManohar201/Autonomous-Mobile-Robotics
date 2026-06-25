#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;
struct Seen { rclcpp::Time t; bool seen{false}; std::string frame; };
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sensor_health_hard");
    auto imu_seen = std::make_shared<Seen>(); auto odom_seen = std::make_shared<Seen>(); auto scan_seen = std::make_shared<Seen>();
    auto mark = [node](std::shared_ptr<Seen> seen, const std::string& frame_id) { seen->t = node->now(); seen->seen = true; seen->frame = frame_id; };
    auto imu = node->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, [=](const sensor_msgs::msg::Imu::SharedPtr msg) { mark(imu_seen, msg->header.frame_id); });
    auto odom = node->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, [=](const nav_msgs::msg::Odometry::SharedPtr msg) { mark(odom_seen, msg->header.frame_id); });
    auto scan = node->create_subscription<sensor_msgs::msg::LaserScan>("/front_laser/scan", 10, [=](const sensor_msgs::msg::LaserScan::SharedPtr msg) { mark(scan_seen, msg->header.frame_id); });
    auto pub = node->create_publisher<std_msgs::msg::String>("/sensor/health", 10);
    auto timer = node->create_wall_timer(500ms, [=] {
        auto stale = [&](const std::shared_ptr<Seen>& s) { return !s->seen || (node->now() - s->t) > rclcpp::Duration::from_seconds(1.0); };
        std_msgs::msg::String out; out.data = (!stale(imu_seen) && !stale(odom_seen) && !stale(scan_seen)) ? "OK" : "STALE_INPUTS"; pub->publish(out);
    });
    rclcpp::spin(node); rclcpp::shutdown(); return 0;
}
