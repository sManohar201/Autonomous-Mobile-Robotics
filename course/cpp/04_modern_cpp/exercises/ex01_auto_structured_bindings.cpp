// Exercise 01 - auto and Structured Bindings
//
// Goal:
//   Use auto intentionally and avoid accidental copies in structured bindings.

#include <cassert>
#include <iostream>
#include <map>
#include <string>
#include <type_traits>

struct SensorInfo {
    std::string frame_id;
    double rate_hz;
};

// TODO 1: Implement total_rate.
// Requirements:
//   - Iterate over map entries with structured bindings.
//   - Avoid copying SensorInfo.
//   - Return the sum of all rate_hz values.

// TODO 2: Implement has_sensor.
// Requirements:
//   - Use auto for the iterator returned by find.
//   - Return true when name exists.

int main() {
    std::map<std::string, SensorInfo> sensors{
        {"imu", {"imu_link", 100.0}},
        {"gps", {"gps_link", 10.0}},
        {"lidar", {"laser", 20.0}},
    };

    // TODO: make these pass.
    // assert(total_rate(sensors) == 130.0);
    // assert(has_sensor(sensors, "imu"));
    // assert(!has_sensor(sensors, "camera"));

    const std::string name = "imu";
    auto copied = name;
    auto& ref = name;
    static_assert(std::is_same_v<decltype(copied), std::string>);
    static_assert(std::is_same_v<decltype(ref), const std::string&>);

    (void)sensors;

    std::cout << "ex01_auto_structured_bindings passed\n";
}

