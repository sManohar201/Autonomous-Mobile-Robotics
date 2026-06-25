// Hard Exercise - Sensor Health Aggregator
// TODO: subscribe to IMU, odom, and scan; track last receive time, frame IDs,
// and publish OK/STALE/FRAME_ERROR health summary.
#include "rclcpp/rclcpp.hpp"
int main(int argc, char** argv) { rclcpp::init(argc, argv); auto node = std::make_shared<rclcpp::Node>("sensor_health_hard"); rclcpp::spin(node); rclcpp::shutdown(); return 0; }

