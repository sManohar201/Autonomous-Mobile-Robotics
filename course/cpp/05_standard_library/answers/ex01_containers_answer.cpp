#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

double mean_range(const std::vector<double>& ranges) {
    if (ranges.empty()) return 0.0;
    double sum = 0.0;
    for (double r : ranges) sum += r;
    return sum / static_cast<double>(ranges.size());
}

double norm3(const std::array<double, 3>& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

std::optional<double> lookup_rate(
    const std::unordered_map<std::string, double>& rates,
    const std::string& name) {
    auto it = rates.find(name);
    if (it == rates.end()) return std::nullopt;
    return it->second;
}

int main() {
    assert(mean_range({1.0, 2.0, 3.0}) == 2.0);
    assert(norm3({3.0, 4.0, 12.0}) == 13.0);
    std::unordered_map<std::string, double> rates{{"imu", 100.0}};
    assert(lookup_rate(rates, "imu").value() == 100.0);
    assert(!lookup_rate(rates, "gps"));
    std::cout << "ex01_containers_answer passed\n";
}

