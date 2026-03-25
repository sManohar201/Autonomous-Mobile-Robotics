#include <sensor_utils/lowpass.hpp>
#include <stdexcept>

namespace sensor_utils {

LowPassFilter::LowPassFilter(double alpha)
    : alpha_(alpha), state_{}, initialized_(false)
{
    if (alpha <= 0.0 || alpha > 1.0) {
        throw std::invalid_argument("LowPassFilter: alpha must be in (0, 1]");
    }
}

robot_math::Vec3 LowPassFilter::update(const robot_math::Vec3& measurement) {
    // Exponential moving average: state = alpha * meas + (1 - alpha) * state
    // state_ starts at {0,0,0}; the filter is applied from the very first call.
    // First call with meas=(0,0,10.5), alpha=0.1:
    //   state = 0.1 * 10.5 + 0.9 * 0 = 1.05
    state_ = robot_math::Vec3{
        alpha_ * measurement.x + (1.0 - alpha_) * state_.x,
        alpha_ * measurement.y + (1.0 - alpha_) * state_.y,
        alpha_ * measurement.z + (1.0 - alpha_) * state_.z
    };
    initialized_ = true;
    return state_;
}

robot_math::Vec3 LowPassFilter::state() const {
    return state_;
}

void LowPassFilter::reset() {
    state_       = robot_math::Vec3{};
    initialized_ = false;
}

} // namespace sensor_utils
