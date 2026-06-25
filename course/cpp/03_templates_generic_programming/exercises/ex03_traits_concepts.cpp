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

template <typename Msg>
struct SensorTraits {
    static constexpr const char* topic = "/unknown";
    static constexpr double default_rate_hz = 1.0;
};

template <>
struct SensorTraits<ImuMsg> {
    static constexpr const char* topic = "/imu/data";
    static constexpr double default_rate_hz = 100.0;
};

template <>
struct SensorTraits<GpsMsg> {
    static constexpr const char* topic = "/gps/fix";
    static constexpr double default_rate_hz = 10.0;
};

template <typename T>
concept HasTimestamp = requires(const T& msg) {
    { msg.timestamp_ns } -> std::convertible_to<std::uint64_t>;
};

template <HasTimestamp Msg>
bool is_newer(const Msg& a, const Msg& b) {
    return a.timestamp_ns > b.timestamp_ns;
}

template <typename Msg>
void print_summary(const Msg& msg) {
    std::cout << SensorTraits<Msg>::topic << " ";
    if constexpr (HasTimestamp<Msg>) {
        std::cout << "timestamp=" << msg.timestamp_ns << "\n";
    } else {
        std::cout << "no timestamp\n";
    }
}

int main() {
    ImuMsg old_imu{100, 0.0, 0.0, 9.8};
    ImuMsg new_imu{200, 0.0, 0.0, 9.7};
    GpsMsg gps{150, 12.0, 77.0, 900.0};
    TextMsg text{"hello"};

    static_assert(HasTimestamp<ImuMsg>);
    static_assert(!HasTimestamp<TextMsg>);
    assert(is_newer(new_imu, old_imu));
    assert(SensorTraits<ImuMsg>::default_rate_hz == 100.0);
    assert(std::string{SensorTraits<GpsMsg>::topic} == "/gps/fix");

    print_summary(new_imu);
    print_summary(gps);
    print_summary(text);

    (void)old_imu;

    std::cout << "ex03_traits_concepts passed\n";
}
