#include <cassert>
#include <iostream>
#include <string>
#include <type_traits>

struct ImuMeasurement {};
struct GpsMeasurement {};
struct UnknownMeasurement {};

template <typename Msg>
std::string frame_id_for() {
    if constexpr (std::is_same_v<Msg, ImuMeasurement>) {
        return "imu_link";
    } else if constexpr (std::is_same_v<Msg, GpsMeasurement>) {
        return "gps_link";
    } else {
        return "base_link";
    }
}

template <typename... Checks>
bool all_true(Checks... checks) {
    return (checks && ...);
}

template <typename... Checks>
bool any_true(Checks... checks) {
    return (checks || ...);
}

template <typename... Values>
auto sum(Values... values) {
    return (values + ...);
}

int main() {
    assert(frame_id_for<ImuMeasurement>() == "imu_link");
    assert(frame_id_for<GpsMeasurement>() == "gps_link");
    assert(frame_id_for<UnknownMeasurement>() == "base_link");

    assert(all_true(true, true, true));
    assert(!all_true(true, false, true));
    assert(any_true(false, false, true));
    assert(sum(1, 2, 3, 4) == 10);

    std::cout << "ex04_constexpr_folds_answer passed\n";
}

