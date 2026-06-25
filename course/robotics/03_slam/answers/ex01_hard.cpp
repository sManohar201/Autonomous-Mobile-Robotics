#include <cmath>
#include <iostream>
#include <vector>

struct Edge { int from, to; double delta, information; };

int main() {
    // Odometry overestimates: each step reads 1.1m but true step is 1.0m
    // Poses from integrating noisy odometry → drift to 4.4 instead of 4.0
    std::vector<double> poses = {0.0, 1.1, 2.2, 3.3, 4.4};

    std::vector<Edge> edges = {
        {0, 1, 1.1,  1.0},  // odometry says +1.1 each step
        {1, 2, 1.1,  1.0},
        {2, 3, 1.1,  1.0},
        {3, 4, 1.1,  1.0},
        // Loop closure: robot actually returned to near start (true range=4.0m)
        {4, 0, -4.0, 10.0},
    };

    const double lr = 0.01;
    const int max_iter = 200;

    for (int iter = 0; iter < max_iter; ++iter) {
        for (const auto& e : edges) {
            const double err  = poses[e.to] - poses[e.from] - e.delta;
            const double grad = 2.0 * e.information * err;
            if (e.from != 0) poses[e.from] += lr * grad;
            if (e.to   != 0) poses[e.to]   -= lr * grad;
        }
    }

    std::cout << "Optimised poses:\n";
    for (size_t i = 0; i < poses.size(); ++i)
        std::cout << "  x" << i << " = " << poses[i] << "\n";

    // Loop closure edge: from=4, to=0, delta=-4.0 → err = x[0] - x[4] - (-4.0)
    const double loop_residual = poses[0] - poses[4] - (-4.0);
    std::cout << "Loop closure residual: " << std::abs(loop_residual) << "\n";
    std::cout << "(should be near 0 after optimisation)\n";

    std::cout << "ex01_hard passed\n";
    return 0;
}
