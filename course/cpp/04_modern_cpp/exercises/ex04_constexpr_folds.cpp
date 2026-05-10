// Exercise 04 - constexpr if and Fold Expressions
//
// Goal:
//   Use compile-time branches and parameter-pack folds for small reusable
//   validation utilities.

#include <cassert>
#include <iostream>
#include <string>
#include <type_traits>

struct ImuMeasurement {};
struct GpsMeasurement {};
struct UnknownMeasurement {};

// TODO 1: Implement frame_id_for<Msg>().
// Use if constexpr:
//   ImuMeasurement -> "imu_link"
//   GpsMeasurement -> "gps_link"
//   anything else -> "base_link"

// TODO 2: Implement all_true(args...) using a fold expression.

// TODO 3: Implement any_true(args...) using a fold expression.

// TODO 4: Implement sum(args...) using a fold expression.

int main() {
    // TODO: make these pass.
    // assert(frame_id_for<ImuMeasurement>() == "imu_link");
    // assert(frame_id_for<GpsMeasurement>() == "gps_link");
    // assert(frame_id_for<UnknownMeasurement>() == "base_link");
    //
    // assert(all_true(true, true, true));
    // assert(!all_true(true, false, true));
    // assert(any_true(false, false, true));
    // assert(sum(1, 2, 3, 4) == 10);

    std::cout << "ex04_constexpr_folds passed\n";
}

