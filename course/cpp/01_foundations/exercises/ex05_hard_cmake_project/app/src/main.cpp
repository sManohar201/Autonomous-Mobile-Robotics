#include <robot_math/vec3.hpp>
#include <sensor_utils/lowpass.hpp>
#include <iostream>

int main() {
    // Direct use of robot_math — note that robot_app must be able to resolve
    // <robot_math/vec3.hpp> even though it only explicitly links sensor_utils.
    // This works because sensor_utils links robot_math PUBLIC, propagating
    // robot_math's include directories to all of sensor_utils's consumers.
    robot_math::Vec3 gravity{0.0, 0.0, 9.81};
    std::cout << "gravity = " << gravity << "\n";

    sensor_utils::LowPassFilter filter(0.1);
    robot_math::Vec3 noisy{0.0, 0.0, 10.5};
    auto filtered = filter.update(noisy);
    std::cout << "filtered = " << filtered << "\n";

    return 0;
}
