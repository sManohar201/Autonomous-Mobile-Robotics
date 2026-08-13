# 6. Environment Variants

Some nodes need to behave differently in `dev`, `sim`, and `field` environments.
The node should never hardcode "if running on hardware, do X" — instead, read
an `environment` parameter and dispatch:

```cpp
bool is_supported_environment(const std::string& env) {
    constexpr std::array<const char*, 3> allowed{"dev", "sim", "field"};
    for (const auto* e : allowed) { if (env == e) return true; }
    return false;
}

std::string lifecycle_policy_for(const std::string& env) {
    if (env == "field") {
        return "configure:drivers,monitoring,localization; activate:all";
    }
    if (env == "sim") {
        return "configure:gazebo_bridge,monitoring,localization; activate:all";
    }
    return "configure:monitoring; activate:monitoring";  // dev
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("variant_bringup_selector");
    node->declare_parameter<std::string>("environment", "dev");
    node->declare_parameter<std::string>("robot_name", "robot");

    const auto env = node->get_parameter("environment").as_string();
    if (!is_supported_environment(env)) {
        RCLCPP_FATAL(node->get_logger(), "Unsupported environment: %s", env.c_str());
        rclcpp::shutdown();
        return 2;
    }
    // ... publish selected profile
}
```

Exit code 2 signals "misconfiguration" — distinct from 1 (runtime error)
and 0 (success). CI pipelines can detect this.
