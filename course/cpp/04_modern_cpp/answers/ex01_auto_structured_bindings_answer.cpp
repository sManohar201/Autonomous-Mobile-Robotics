#include <cassert>
#include <iostream>
#include <map>
#include <string>
#include <type_traits>

struct SensorInfo {
    std::string frame_id;
    double rate_hz;
};

double total_rate(const std::map<std::string, SensorInfo>& sensors) {
    double total = 0.0;
    for (const auto& [name, info] : sensors) {
        (void)name;
        total += info.rate_hz;
    }
    return total;
}

bool has_sensor(const std::map<std::string, SensorInfo>& sensors,
                const std::string& name) {
    auto it = sensors.find(name);
    return it != sensors.end();
}

int main() {
    std::map<std::string, SensorInfo> sensors{
        {"imu", {"imu_link", 100.0}},
        {"gps", {"gps_link", 10.0}},
        {"lidar", {"laser", 20.0}},
    };

    assert(total_rate(sensors) == 130.0);
    assert(has_sensor(sensors, "imu"));
    assert(!has_sensor(sensors, "camera"));

    const std::string name = "imu";
    auto copied = name;
    auto& ref = name;
    static_assert(std::is_same_v<decltype(copied), std::string>);
    static_assert(std::is_same_v<decltype(ref), const std::string&>);

    std::cout << "ex01_auto_structured_bindings_answer passed\n";
}

