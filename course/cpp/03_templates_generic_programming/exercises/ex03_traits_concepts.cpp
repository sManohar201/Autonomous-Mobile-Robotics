// Exercise 03 - Traits, if constexpr, and Concepts
//
// Goal:
//   Write generic message utilities constrained by compile-time requirements.

#include <cassert>
#include <concepts>
#include <cstdint>
#include <iostream>
#include <string>
#include <type_traits>

struct ImuMsg {
    std::uint64_t timestamp_ns;
    double ax, ay, az;
};

struct GpsMsg {
    std::uint64_t timestamp_ns;
    double lat, lon, alt;
};

struct TextMsg {
    std::string text;
};

// TODO 1: Define SensorTraits primary template.
// Default topic should be "/unknown" and default_rate_hz should be 1.0.

// TODO 2: Specialize SensorTraits<ImuMsg>.
// topic: "/imu/data", default_rate_hz: 100.0

// TODO 3: Specialize SensorTraits<GpsMsg>.
// topic: "/gps/fix", default_rate_hz: 10.0

// TODO 4: Define HasTimestamp concept.
// It should require msg.timestamp_ns convertible to std::uint64_t.

// TODO 5: Implement is_newer(a, b), constrained with HasTimestamp.

// TODO 6: Implement print_summary(msg).
// Use if constexpr:
//   - If HasTimestamp<Msg>, print topic and timestamp.
//   - Otherwise, print topic and "no timestamp".

int main() {
    ImuMsg old_imu{100, 0.0, 0.0, 9.8};
    ImuMsg new_imu{200, 0.0, 0.0, 9.7};
    GpsMsg gps{150, 12.0, 77.0, 900.0};
    TextMsg text{"hello"};

    // TODO: make these pass.
    // static_assert(HasTimestamp<ImuMsg>);
    // static_assert(!HasTimestamp<TextMsg>);
    // assert(is_newer(new_imu, old_imu));
    // assert(SensorTraits<ImuMsg>::default_rate_hz == 100.0);
    // assert(std::string{SensorTraits<GpsMsg>::topic} == "/gps/fix");

    (void)old_imu;
    (void)new_imu;
    (void)gps;
    (void)text;

    std::cout << "ex03_traits_concepts passed\n";
}

