// main.cpp — 1D Kalman Filter integration test (answer version)
// Uses the answer header (kalman1d.hpp in the same directory).

#include "kalman1d.hpp"
#include <iostream>
#include <iomanip>
#include <vector>

int main() {
    std::cout << std::fixed << std::setprecision(4);

    // ── 1. Direct Gaussian fusion ─────────────────────────────────────────────
    std::cout << "--- direct Gaussian fusion ---\n";

    bayes::GaussianState g1(1.0, 1.0);
    bayes::GaussianState g2(2.0, 1.0);
    bayes::GaussianState fused = g1 * g2;

    std::cout << g1 << " * " << g2 << " = " << fused << "\n";

    std::cout << "Verify: fused var (" << fused.variance << ") < both inputs (1.0): "
              << ((fused.variance < g1.variance && fused.variance < g2.variance)
                  ? "PASS" : "FAIL") << "\n";

    bayes::GaussianState fused2 = g2 * g1;
    std::cout << "Symmetric: g1*g2 == g2*g1: "
              << ((fused == fused2) ? "PASS" : "FAIL") << "\n\n";

    // ── 2. Predict loop ───────────────────────────────────────────────────────
    std::cout << "--- predict loop (5 steps, dt=0.1s, v=1.0 m/s) ---\n";

    bayes::GaussianState initial(0.0, 1.0);
    bayes::KalmanFilter1D kf(initial, 0.1);

    bayes::FilterHistory history(10);

    const double dt = 0.1;
    const double v  = 1.0;

    for (int i = 0; i < 5; ++i) {
        {
            bayes::ScopedFilterLogger logger("predict", kf);
            kf.predict(v, dt);
        }
        history.record(kf.state());
    }
    std::cout << "\n";

    // ── 3. Print history ──────────────────────────────────────────────────────
    std::cout << "--- filter history ---\n";
    for (int i = 0; i < history.size(); ++i) {
        std::cout << "history[" << i << "]: " << history.at(i) << "\n";
    }
    std::cout << "\n";

    bool all_grew = true;
    for (int i = 0; i < history.size(); ++i) {
        if (history.at(i).variance <= initial.variance) {
            all_grew = false;
        }
    }
    std::cout << "All predicted variances > initial: " << (all_grew ? "PASS" : "FAIL") << "\n\n";

    // ── 4. Update loop ────────────────────────────────────────────────────────
    std::cout << "--- update loop (3 sensor measurements) ---\n";

    std::vector<bayes::GaussianState> measurements = {
        bayes::GaussianState(0.3, 0.5),
        bayes::GaussianState(0.6, 0.3),
        bayes::GaussianState(1.1, 0.2),
    };

    double var_before_updates = kf.state().variance;

    for (const auto& meas : measurements) {
        {
            bayes::ScopedFilterLogger logger("update", kf);
            kf.update(meas);
        }
    }
    std::cout << "\n";

    double var_after_updates = kf.state().variance;
    std::cout << "Final variance < variance before updates ("
              << var_before_updates << "): "
              << ((var_after_updates < var_before_updates) ? "PASS" : "FAIL") << "\n";

    // ── 5. Final state ────────────────────────────────────────────────────────
    std::cout << "\n--- final state ---\n";
    kf.log("final");

    // ── 6. Exception handling ─────────────────────────────────────────────────
    std::cout << "\n--- exception handling ---\n";

    try {
        bayes::GaussianState bad(0.0, 0.0);
    } catch (const std::invalid_argument& e) {
        std::cout << "[exception] zero variance: " << e.what() << "\n";
    }

    try {
        bayes::GaussianState bad(0.0, -1.0);
    } catch (const std::invalid_argument& e) {
        std::cout << "[exception] negative variance: " << e.what() << "\n";
    }

    try {
        bayes::KalmanFilter1D bad_kf(initial, -0.1);
    } catch (const std::invalid_argument& e) {
        std::cout << "[exception] negative process noise: " << e.what() << "\n";
    }

    // ── 7. FilterHistory Rule of Five ─────────────────────────────────────────
    std::cout << "\n--- FilterHistory copy/move ---\n";

    bayes::FilterHistory h2 = history;
    std::cout << "copied history size: " << h2.size() << "\n";

    h2.record(bayes::GaussianState(99.0, 99.0));
    std::cout << "original history size after h2 modification: "
              << history.size() << " (should be " << history.size() << ")\n";

    bayes::FilterHistory h3 = std::move(h2);
    std::cout << "moved history size: " << h3.size() << "\n";
    std::cout << "source after move size: " << h2.size() << " (should be 0)\n";

    // ── 8. FilterHistory overflow ─────────────────────────────────────────────
    std::cout << "\n--- FilterHistory overflow (capacity=3) ---\n";
    bayes::FilterHistory small(3);
    for (int i = 0; i < 5; ++i) {
        small.record(bayes::GaussianState(static_cast<double>(i), 1.0));
    }
    std::cout << "size after 5 inserts into cap-3 buffer: " << small.size()
              << " (expected 3)\n";
    std::cout << "oldest (i=2): " << small.at(0) << "\n";
    std::cout << "newest (i=4): " << small.at(2) << "\n";

    return 0;
}
