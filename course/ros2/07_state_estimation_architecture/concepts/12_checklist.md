# 11. Checklist

- [ ] Adapter is a pure function — no global state, no ROS calls
- [ ] All three rejection conditions are implemented and logged
- [ ] Reset clears state AND counters (accepted, rejected, initialized)
- [ ] Diagnostics include `update_age_s` — detects silent measurement loss
- [ ] Reset service returns meaningful `message` field for all outcomes
- [ ] Filter core (StateEstimator) has no `#include "rclcpp/rclcpp.hpp"` — keeps it testable in isolation
