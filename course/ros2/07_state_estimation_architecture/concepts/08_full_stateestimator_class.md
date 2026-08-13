# 7. Full StateEstimator Class

```cpp
class StateEstimator {
public:
    explicit StateEstimator(std::string required_frame)
        : required_frame_(std::move(required_frame)) {}

    void predict(const rclcpp::Time& stamp) {
        last_predict_stamp_ = stamp;
    }

    bool update(const nav_msgs::msg::Odometry& msg, std::string& reason) {
        if (msg.header.frame_id != required_frame_) {
            reason = "wrong_frame"; ++rejected_; return false;
        }
        const double cov = msg.pose.covariance[0]
                         + msg.pose.covariance[7]
                         + msg.pose.covariance[35];
        if (cov > max_cov_trace_) {
            reason = "high_covariance"; ++rejected_; return false;
        }
        const double dx = msg.pose.pose.position.x - x_;
        const double dy = msg.pose.pose.position.y - y_;
        if (initialized_ && std::hypot(dx, dy) > max_jump_m_) {
            reason = "outlier_jump"; ++rejected_; return false;
        }
        x_ = msg.pose.pose.position.x;
        y_ = msg.pose.pose.position.y;
        last_update_stamp_ = msg.header.stamp;
        initialized_ = true;
        ++accepted_;
        reason = "accepted";
        return true;
    }

    void reset() {
        x_ = 0.0; y_ = 0.0;
        accepted_ = 0; rejected_ = 0;
        initialized_ = false;
    }

    std::string diagnostics(const rclcpp::Clock& clock) const {
        std::ostringstream out;
        out << "initialized=" << (initialized_ ? "true" : "false")
            << " accepted=" << accepted_
            << " rejected=" << rejected_
            << " predict_age_s=" << (clock.now() - last_predict_stamp_).seconds()
            << " update_age_s="  << (clock.now() - last_update_stamp_).seconds();
        return out.str();
    }

private:
    std::string required_frame_;
    double max_cov_trace_{1.0};
    double max_jump_m_{3.0};
    double x_{0.0}, y_{0.0};
    int accepted_{0}, rejected_{0};
    bool initialized_{false};
    rclcpp::Time last_predict_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_update_stamp_{0, 0, RCL_ROS_TIME};
};
```
