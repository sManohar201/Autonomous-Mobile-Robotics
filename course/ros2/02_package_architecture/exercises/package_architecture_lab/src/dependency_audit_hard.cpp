// Hard Exercise - Dependency Audit
//
// Model a set of ROS2 package dependencies in code and detect policy violations:
//   - interfaces packages must not depend on application packages
//   - description packages must not depend on navigation packages
//
// Publish the audit result on /architecture/dependency_audit (std_msgs/String):
//   "dependency audit OK"  — if no violations found
//   "forbidden A->B ..."   — listing each violation
//
// Design your own data structures for the dependency graph.

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // TODO
    rclcpp::shutdown();
    return 0;
}
