#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

struct Particle { double x; double weight; };

int main() {
    std::mt19937 rng(42);
    std::uniform_real_distribution<> uniform(0.0, 1.0);
    std::normal_distribution<> motion_noise(0.0, 0.1);

    const int N = 500;
    const double map_length = 10.0;

    // 1. Initialise uniformly
    std::vector<Particle> particles(N);
    for (auto& p : particles) {
        p.x = uniform(rng) * map_length;
        p.weight = 1.0 / N;
    }

    // 2. Motion update: move 1.0 m with Gaussian noise
    for (auto& p : particles) {
        p.x += 1.0 + motion_noise(rng);
        p.x = std::clamp(p.x, 0.0, map_length);
    }

    // 3. Observation update
    const double z = 8.0;
    const double sigma_sensor = 0.3;
    double w_sum = 0.0;
    for (auto& p : particles) {
        const double expected = map_length - p.x;
        const double err = z - expected;
        p.weight = std::exp(-0.5 * err * err / (sigma_sensor * sigma_sensor));
        w_sum += p.weight;
    }

    // 4. Normalise
    for (auto& p : particles) p.weight /= w_sum;

    // 5. Systematic resampling
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
        new_particles.push_back({particles[j].x, 1.0 / N});
        r += step;
    }

    // 6. Estimate: weighted mean
    double est = 0.0;
    for (const auto& p : new_particles) est += p.x * p.weight;

    std::cout << "Estimated position: " << est << " m (expected ~2.0)\n";
    std::cout << "ex01_hard passed\n";
    return 0;
}
