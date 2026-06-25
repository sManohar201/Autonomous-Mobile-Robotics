// Hard Exercise — 1D Particle Filter (Monte Carlo Localization)
//
// Implement a particle filter that localises a robot on a 1D corridor of length 10m.
//
// Setup:
//   - N=500 particles, each is a position hypothesis x ∈ [0, 10]
//   - Robot has a distance sensor that measures distance to the right wall
//   - Map: right wall is at x=10
//
// Steps:
//   1. Init: distribute N particles uniformly on [0, 10], weight = 1/N
//   2. Motion update: robot moves forward by delta=1.0 m with Gaussian noise σ=0.1
//   3. Observation update: sensor reads z=8.0 m to right wall
//      expected measurement: 10 - particle.x
//      weight ← exp(-(z - expected)² / (2 * σ_sensor²)),  σ_sensor = 0.3
//   4. Normalise weights
//   5. Resample: systematic resampling
//   6. Estimate: weighted mean
//   7. Print estimated position (should be near 2.0)
//
// No hints about implementation — design the data structure and algorithm yourself.

#include <cmath>
#include <iostream>
#include <random>
#include <vector>

int main() {
    // TODO: implement particle filter

    std::cout << "ex01_hard passed\n";
    return 0;
}
