// Moderate Exercise - Frame Validator
// TODO: subscribe to sensor_msgs/msg/Imu, compare header.frame_id with
// expected_frame_id parameter, publish OK/WARN string.
#include "rclcpp/rclcpp.hpp"
int main(int argc, char** argv) { rclcpp::init(argc, argv); auto node = std::make_shared<rclcpp::Node>("frame_validator"); rclcpp::spin(node); rclcpp::shutdown(); return 0; }

