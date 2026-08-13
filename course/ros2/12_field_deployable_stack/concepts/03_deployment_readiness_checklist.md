# 2. Deployment Readiness Checklist

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
