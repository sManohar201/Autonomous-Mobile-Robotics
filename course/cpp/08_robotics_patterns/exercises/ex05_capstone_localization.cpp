#include <iostream>
#include <vector>

struct State1d {
    double x;
    double v;
};

// TODO: implement MotionModel, MeasurementModel, and Localizer1d.
// predict(dt): x += v * dt
// update(position): blend current x toward measurement.
// Store history after each predict/update.

int main() {
    std::cout << "ex05_capstone_localization passed\n";
}

