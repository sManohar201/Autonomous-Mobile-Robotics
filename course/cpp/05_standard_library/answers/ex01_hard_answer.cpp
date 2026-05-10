#include <cassert>
#include <iostream>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

struct Sensor { std::string name; double rate_hz; };

void append_batch(std::vector<Sensor>& dst, const std::vector<Sensor>& src) {
    dst.reserve(dst.size() + src.size());
    for (const auto& sensor : src) dst.push_back(sensor);
}

std::optional<double> lookup_rate(const std::unordered_map<std::string, Sensor>& sensors,
                                  const std::string& name) {
    auto it = sensors.find(name);
    if (it == sensors.end()) return std::nullopt;
    return it->second.rate_hz;
}

int main() {
    std::vector<Sensor> sensors{{"imu", 100.0}};
    append_batch(sensors, {{"gps", 10.0}, {"lidar", 20.0}});
    assert(sensors.size() == 3);
    std::unordered_map<std::string, Sensor> map{{"imu", {"imu", 100.0}}};
    assert(lookup_rate(map, "imu").value() == 100.0);
    std::cout << "ex01_hard_answer passed\n";
}

