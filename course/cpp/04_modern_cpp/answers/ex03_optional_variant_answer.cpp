#include <cassert>
#include <cmath>
#include <optional>
#include <iostream>
#include <string>
#include <variant>

std::optional<double> parse_double(const std::string& text) {
    try {
        std::size_t pos = 0;
        const double value = std::stod(text, &pos);
        if (pos != text.size()) {
            return std::nullopt;
        }
        return value;
    } catch (...) {
        return std::nullopt;
    }
}

struct ImuMeasurement {
    double ax, ay, az;
};

struct GpsMeasurement {
    double lat, lon, alt;
};

struct LidarMeasurement {
    double range_m;
};

using Measurement = std::variant<ImuMeasurement, GpsMeasurement, LidarMeasurement>;

template <class... Ts>
struct Overload : Ts... {
    using Ts::operator()...;
};
template <class... Ts>
Overload(Ts...) -> Overload<Ts...>;

std::string measurement_name(const Measurement& m) {
    return std::visit(Overload{
        [](const ImuMeasurement&) { return std::string{"imu"}; },
        [](const GpsMeasurement&) { return std::string{"gps"}; },
        [](const LidarMeasurement&) { return std::string{"lidar"}; },
    }, m);
}

double measurement_quality(const Measurement& m) {
    return std::visit(Overload{
        [](const ImuMeasurement& imu) { return std::abs(imu.az); },
        [](const GpsMeasurement& gps) { return gps.alt; },
        [](const LidarMeasurement& lidar) { return lidar.range_m; },
    }, m);
}

int main() {
    assert(parse_double("3.5").value() == 3.5);
    assert(!parse_double("3.5abc"));

    Measurement m = ImuMeasurement{0.0, 0.1, 9.8};
    assert(measurement_name(m) == "imu");
    assert(measurement_quality(m) == 9.8);

    m = LidarMeasurement{12.0};
    assert(measurement_name(m) == "lidar");
    assert(measurement_quality(m) == 12.0);

    std::cout << "ex03_optional_variant_answer passed\n";
}

