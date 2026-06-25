// Exercise 01 (Hard) - Iterator Invalidation and Container Choice
// Tasks: diagnose vector/deque/map invalidation cases; implement safe append
// with reserve; implement stable sensor lookup without dangling references.
//
// Iterator invalidation rules:
//   vector: push_back invalidates ALL iterators if reallocation occurs.
//           Use reserve() to prevent reallocation and preserve iterators.
//   deque:  push_back/push_front invalidates iterators but NOT references.
//   map:    insertions NEVER invalidate existing iterators or references.
//   unordered_map: rehash invalidates ALL iterators; references stay valid.

#include <cassert>
#include <iostream>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

struct Sensor { std::string name; double rate_hz; };

void append_batch(std::vector<Sensor>& dst, const std::vector<Sensor>& src) {
    dst.reserve(dst.size() + src.size());
    for (const auto& s : src) dst.push_back(s);
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
    assert(!lookup_rate(map, "gps"));

    std::cout << "ex01_hard passed\n";
}

