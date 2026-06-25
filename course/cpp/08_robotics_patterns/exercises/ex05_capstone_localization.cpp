#include <iostream>
#include <vector>

struct State1d { double x, v; };

// Exercise: implement a 1D localization pipeline with three components.
//
// MotionModel — predict(state, dt): returns new state with x += v*dt
//
// MeasurementModel(gain) — update(state, z): blends x toward measurement:
//                           x += gain * (z - x)
//
// Localizer1d(initial_state, gain):
//   - stores history starting with the initial state
//   - predict(dt) applies the motion model and appends to history
//   - update(z)   applies the measurement model and appends to history
//   - state()     returns the current state
//   - history()   returns a const reference to all recorded states

int main() {
    // TODO
    std::cout << "ex05_capstone_localization passed\n";
}
