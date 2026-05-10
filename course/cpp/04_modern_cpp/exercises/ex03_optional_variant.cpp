// Exercise 03 - optional and variant
//
// Goal:
//   Represent missing values and closed sets of measurement types explicitly.

#include <cassert>
#include <optional>
#include <iostream>
#include <string>
#include <variant>

// TODO 1: Implement parse_double.
// Return std::nullopt unless the full string parses as a double.

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

// TODO 2: Add Overload helper for std::visit.

// TODO 3: Implement measurement_name.
// Return "imu", "gps", or "lidar".

// TODO 4: Implement measurement_quality.
// Simple rules:
//   - IMU quality is absolute az.
//   - GPS quality is altitude.
//   - LiDAR quality is range_m.

int main() {
    // TODO: make these pass.
    // assert(parse_double("3.5").value() == 3.5);
    // assert(!parse_double("3.5abc"));
    //
    // Measurement m = ImuMeasurement{0.0, 0.1, 9.8};
    // assert(measurement_name(m) == "imu");
    // assert(measurement_quality(m) == 9.8);
    //
    // m = LidarMeasurement{12.0};
    // assert(measurement_name(m) == "lidar");
    // assert(measurement_quality(m) == 12.0);

    std::cout << "ex03_optional_variant passed\n";
}

