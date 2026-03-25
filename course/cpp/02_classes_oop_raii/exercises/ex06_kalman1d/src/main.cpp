// main.cpp — 1D Kalman Filter integration test
//
// This main() exercises every component from kalman1d.hpp.
// Read through it completely before starting to implement any functions —
// understanding what main() expects will guide your implementation decisions.
//
// What this program demonstrates
// --------------------------------
// 1. GaussianState arithmetic: fusing two independent position measurements
//    directly (the "update only" Bayes rule).
//
// 2. KalmanFilter1D predict-only loop: variance grows monotonically.
//    Intuition: the robot is moving but has no sensors — uncertainty increases.
//
// 3. KalmanFilter1D update loop: applying three sensor measurements.
//    Each update shrinks the variance.  After three measurements from
//    sensors with low variance, the filter is much more confident than
//    after the motion-only phase.
//
// 4. ScopedFilterLogger: wrapping each predict/update so that before/after
//    state transitions are logged to stdout.
//
// 5. FilterHistory: storing snapshots after each predict step and printing
//    the history at the end.
//
// ── Expected output structure ────────────────────────────────────────────────
//
// --- direct Gaussian fusion ---
// N(mean=1.0000, var=1.0000) * N(mean=2.0000, var=1.0000) = N(mean=1.5000, var=0.5000)
// Verify: fused var (0.5) < both inputs (1.0): PASS
//
// --- predict loop (5 steps, dt=0.1s, v=1.0 m/s) ---
// [predict] before: N(mean=0.0000, var=1.0000)
//   after: N(mean=0.1000, var=1.0010)
// [predict] before: N(mean=0.1000, var=1.0010)
//   after: N(mean=0.2000, var=1.0020)
// ... (5 lines total)
//
// --- filter history ---
// history[0]: N(mean=0.1000, var=1.0010)
// history[1]: N(mean=0.2000, var=1.0020)
// ...
//
// --- update loop (3 sensor measurements) ---
// [update] before: N(mean=0.5000, var=1.0050)
//   after: N(mean=~0.38, var=~0.34)    [approximate]
// ...
//
// --- final state ---
// N(mean=~0.85, var=~0.09)    [approximate — depends on fusion math]
//
// --- invariant checks ---
// All predicted variances > initial: PASS
// Final variance < predicted variance: PASS

#include "kalman1d.hpp"
#include <iostream>
#include <iomanip>
#include <cassert>
#include <vector>

int main() {
    std::cout << std::fixed << std::setprecision(4);

    // ── 1. Direct Gaussian fusion ─────────────────────────────────────────────
    // Fuse two independent position measurements:
    //   Sensor A says: position ≈ 1.0 m, variance = 1.0
    //   Sensor B says: position ≈ 2.0 m, variance = 1.0
    // The fused estimate should be at the midpoint (1.5 m) with variance 0.5.
    //
    // This is the SENSOR FUSION use case — combining two independent sensors.
    // The Kalman update step does exactly this between the prior belief and
    // a new measurement.
    std::cout << "--- direct Gaussian fusion ---\n";

    bayes::GaussianState g1(1.0, 1.0);
    bayes::GaussianState g2(2.0, 1.0);
    bayes::GaussianState fused = g1 * g2;

    std::cout << g1 << " * " << g2 << " = " << fused << "\n";

    // Invariant check: fused variance must be LESS than both inputs.
    std::cout << "Verify: fused var (" << fused.variance << ") < both inputs (1.0): "
              << ((fused.variance < g1.variance && fused.variance < g2.variance)
                  ? "PASS" : "FAIL") << "\n";

    // Symmetric check: g1 * g2 == g2 * g1
    bayes::GaussianState fused2 = g2 * g1;
    std::cout << "Symmetric: g1*g2 == g2*g1: "
              << ((fused == fused2) ? "PASS" : "FAIL") << "\n\n";

    // ── 2. Predict loop ───────────────────────────────────────────────────────
    // Robot starts at position 0.0 m, variance 1.0 m² (uncertain).
    // Process noise: q = 0.1 (mild uncertainty in velocity command).
    // Robot moves forward at v = 1.0 m/s for 5 steps of dt = 0.1 s.
    //
    // After 5 steps: expected position ≈ 0.5 m, variance ≈ 1.0 + 5*(0.1*0.01)
    //   = 1.0 + 0.005 = 1.005 (uncertainty grew slightly).
    std::cout << "--- predict loop (5 steps, dt=0.1s, v=1.0 m/s) ---\n";

    bayes::GaussianState initial(0.0, 1.0);
    bayes::KalmanFilter1D kf(initial, 0.1);  // process_noise = 0.1

    bayes::FilterHistory history(10);  // capacity 10

    const double dt = 0.1;
    const double v  = 1.0;

    for (int i = 0; i < 5; ++i) {
        // ScopedFilterLogger prints before-state on construction, after-state on destruction.
        // The scope ends AFTER filter.predict() so the destructor sees the new state.
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

    // Check: all variances in history must exceed the initial variance.
    bool all_grew = true;
    for (int i = 0; i < history.size(); ++i) {
        if (history.at(i).variance <= initial.variance) {
            all_grew = false;
        }
    }
    std::cout << "All predicted variances > initial: " << (all_grew ? "PASS" : "FAIL") << "\n\n";

    // ── 4. Update loop ────────────────────────────────────────────────────────
    // Apply three sensor measurements.  The robot has been predicted to ~0.5 m.
    // Measurements come from a sensor with varying noise levels.
    //
    // After each update the variance should shrink.
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

    // GaussianState with zero variance must throw.
    try {
        bayes::GaussianState bad(0.0, 0.0);
    } catch (const std::invalid_argument& e) {
        std::cout << "[exception] zero variance: " << e.what() << "\n";
    }

    // GaussianState with negative variance must throw.
    try {
        bayes::GaussianState bad(0.0, -1.0);
    } catch (const std::invalid_argument& e) {
        std::cout << "[exception] negative variance: " << e.what() << "\n";
    }

    // KalmanFilter1D with negative process noise must throw.
    try {
        bayes::KalmanFilter1D bad_kf(initial, -0.1);
    } catch (const std::invalid_argument& e) {
        std::cout << "[exception] negative process noise: " << e.what() << "\n";
    }

    // ── 7. FilterHistory Rule of Five ─────────────────────────────────────────
    std::cout << "\n--- FilterHistory copy/move ---\n";

    bayes::FilterHistory h2 = history;   // copy constructor
    std::cout << "copied history size: " << h2.size() << "\n";

    // Verify independence: recording into h2 should not affect history.
    h2.record(bayes::GaussianState(99.0, 99.0));
    std::cout << "original history size after h2 modification: "
              << history.size() << " (should be " << history.size() << ")\n";

    bayes::FilterHistory h3 = std::move(h2);   // move constructor
    std::cout << "moved history size: " << h3.size() << "\n";
    std::cout << "source after move size: " << h2.size() << " (should be 0)\n";

    // ── 8. FilterHistory overflow ─────────────────────────────────────────────
    std::cout << "\n--- FilterHistory overflow (capacity=3) ---\n";
    bayes::FilterHistory small(3);
    for (int i = 0; i < 5; ++i) {
        small.record(bayes::GaussianState(static_cast<double>(i), 1.0));
    }
    // After 5 inserts into capacity-3 buffer, we should have 3 entries,
    // oldest being i=2, newest being i=4.
    std::cout << "size after 5 inserts into cap-3 buffer: " << small.size()
              << " (expected 3)\n";
    std::cout << "oldest (i=2): " << small.at(0) << "\n";
    std::cout << "newest (i=4): " << small.at(2) << "\n";

    return 0;
}
