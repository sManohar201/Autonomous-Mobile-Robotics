# 5. Reset Service

A `std_srvs/Trigger` service is the standard interface for resetting a ROS2
subsystem. It takes no parameters and returns `{success, message}`.

```cpp
#include "std_srvs/srv/trigger.hpp"

auto reset_srv = node->create_service<std_srvs::srv::Trigger>(
    "localization/reset",
    [estimator](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        estimator->reset();
        resp->success = true;
        resp->message = "estimator reset";
    });
```

Call from the command line:
```bash
ros2 service call /localization/reset std_srvs/srv/Trigger
```

### What reset() should do

```cpp
void reset() {
    x_ = 0.0;
    y_ = 0.0;
    accepted_ = 0;
    rejected_ = 0;
    initialized_ = false;
    // Real filter: also reset covariance to initial P0
}
```

> **Production consideration:** Should you allow reset while the robot is moving?
> Some robots reject reset requests if velocity exceeds a threshold — the filter
> would immediately receive a large jump from the current motion state.
> Extend the service to check velocity and return `success=false` with a
> descriptive message if unsafe.
