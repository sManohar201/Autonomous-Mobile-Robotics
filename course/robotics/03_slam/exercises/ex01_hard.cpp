// Hard Exercise — 1D Pose Graph SLAM with Loop Closure
//
// Implement a 1D pose graph SLAM system.
//
// Setup:
//   - Robot visits 5 poses: x0, x1, x2, x3, x4
//   - Odometry overestimates: each step reads +1.1m but true step is 1.0m
//   - Poses initialised from odometry: x0=0, x1=1.1, x2=2.2, x3=3.3, x4=4.4
//   - Odometry edges: all (delta=+1.1, information=1.0)
//   - Loop closure: robot at x4 recognises x0; true distance = 4.0m → delta=-4.0, info=10.0
//
// Task:
//   1. Initialise poses: {0.0, 1.1, 2.2, 3.3, 4.4}
//   2. Add edges: 4 odometry edges + 1 loop closure
//   3. Optimise: gradient descent to minimise Σ info * (x[j]-x[i]-delta)²
//      - Fix x0=0.0 (gauge freedom)
//      - Run for 200 iterations with lr=0.01
//   4. Print final poses and the loop closure residual (should be near 0)
//
// Expected result: poses should be nearly evenly spaced (~1.0m apart)
//   x0≈0.0, x1≈1.0, x2≈2.0, x3≈3.0, x4≈4.0
//
// No hints — design the data structure and optimisation loop yourself.

#include <iostream>
#include <vector>

int main() {
    // TODO: implement pose graph SLAM

    std::cout << "ex01_hard passed\n";
    return 0;
}
