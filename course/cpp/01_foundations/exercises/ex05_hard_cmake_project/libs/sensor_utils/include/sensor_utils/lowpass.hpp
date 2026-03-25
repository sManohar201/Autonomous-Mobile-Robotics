#pragma once
#include <robot_math/vec3.hpp>

// NOTE: This header includes <robot_math/vec3.hpp>.
// Any target that links sensor_utils and includes this header also needs
// robot_math's include directory to resolve that nested include.
// In CMake, this is handled by linking robot_math with the PUBLIC keyword
// so that sensor_utils's consumers automatically receive robot_math's
// include directories as a transitive dependency.

namespace sensor_utils {

class LowPassFilter {
public:
    // alpha: smoothing factor in (0, 1].
    //   alpha = 1.0  → output equals new measurement immediately (no smoothing)
    //   alpha → 0.0  → very heavy smoothing (slow to respond)
    explicit LowPassFilter(double alpha);

    // Update with a new measurement and return the filtered value.
    // EMA is applied from the first call; state starts at {0,0,0}.
    //   state = alpha * measurement + (1 - alpha) * state
    robot_math::Vec3 update(const robot_math::Vec3& measurement);

    robot_math::Vec3 state() const;
    void reset();

private:
    double           alpha_;
    robot_math::Vec3 state_;
    bool             initialized_;
};

} // namespace sensor_utils
