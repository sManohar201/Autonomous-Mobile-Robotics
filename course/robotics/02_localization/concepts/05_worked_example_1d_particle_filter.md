# 4. Worked Example: 1D Particle Filter

```cpp
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

struct Particle { double x; double weight; };

int main() {
    std::mt19937 rng(42);
    std::uniform_real_distribution<> uniform(0.0, 1.0);
    std::normal_distribution<> gauss(0.0, 1.0);

    const int N = 500;
    const double map_length = 10.0;

    // 1. Initialize uniformly — global localization
    std::vector<Particle> particles(N);
    for (auto& p : particles) {
        p.x = uniform(rng) * map_length;
        p.weight = 1.0 / N;
    }

    // 2. Motion update: robot moves 1.0 m with 0.1 m std noise
    for (auto& p : particles) {
        p.x += 1.0 + gauss(rng) * 0.1;
        p.x = std::clamp(p.x, 0.0, map_length);
    }

    // 3. Observation update: measured distance to wall = 8.0 m
    //    True position is ~ 2.0 m from left wall
    const double measured_dist = 8.0;  // distance to right wall
    const double sigma_sensor = 0.3;
    double weight_sum = 0.0;
    for (auto& p : particles) {
        const double expected_dist = map_length - p.x;
        const double err = measured_dist - expected_dist;
        p.weight = std::exp(-0.5 * err * err / (sigma_sensor * sigma_sensor));
        weight_sum += p.weight;
    }
    for (auto& p : particles) p.weight /= weight_sum;

    // 4. Resample (systematic)
    std::vector<double> cdf(N);
    cdf[0] = particles[0].weight;
    for (int i = 1; i < N; ++i) cdf[i] = cdf[i-1] + particles[i].weight;
    const double step = 1.0 / N;
    double r = uniform(rng) * step;
    std::vector<Particle> new_particles;
    new_particles.reserve(N);
    int j = 0;
    for (int i = 0; i < N; ++i) {
        while (r > cdf[j] && j < N-1) ++j;
        new_particles.push_back(particles[j]);
        new_particles.back().weight = 1.0 / N;
        r += step;
    }

    // 5. Estimate: weighted mean
    double est = 0.0;
    for (const auto& p : new_particles) est += p.x * p.weight;

    std::cout << "Estimated position: " << est << " m\n";
    std::cout << "True position: 2.0 m\n";

    return 0;
}
```
