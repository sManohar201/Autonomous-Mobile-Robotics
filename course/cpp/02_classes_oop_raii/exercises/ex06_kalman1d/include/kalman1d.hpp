// kalman1d.hpp — 1D Kalman Filter: Module 2 Capstone
//
// This header integrates every concept from Module 2 into a genuine Bayesian
// filter implementation.
//
// Mathematical background
// -----------------------
// A 1D Kalman filter tracks a Gaussian belief over a scalar state (e.g., the
// position of a robot along a corridor).
//
// State representation:
//   Belief at time k: p(x_k | z_{1:k}) = N(μ_k, σ²_k)
//   A Gaussian is fully characterised by its mean μ and variance σ².
//
// Predict step (process model: constant velocity):
//   x_{k+1} = x_k + v * dt  (v = commanded velocity, dt = time step)
//   μ_{k+1} = μ_k + v * dt
//   σ²_{k+1} = σ²_k + q * dt²    (q = process noise variance per s²)
//   The variance GROWS during predict — the filter becomes less certain
//   because the motion model is not perfect.
//
// Update step (sensor measurement z ~ N(z_mean, z_var)):
//   The Bayes posterior of two independent Gaussians is:
//     N(μ_1, σ²_1) * N(μ_2, σ²_2) = N(μ_fused, σ²_fused)
//   where:
//     μ_fused  = (μ_1 * σ²_2 + μ_2 * σ²_1) / (σ²_1 + σ²_2)
//     σ²_fused = (σ²_1 * σ²_2) / (σ²_1 + σ²_2)
//   The variance SHRINKS — measurements reduce uncertainty.
//   Note: σ²_fused < min(σ²_1, σ²_2) — the fused result is more certain
//   than either individual estimate alone.
//
// Key invariant: variance > 0 always.
//   Predict adds positive noise (q*dt² > 0), so variance stays positive.
//   Update fuses two positive variances: the harmonic-mean formula keeps
//   the result positive.  But you must ensure the initial variance is > 0,
//   and throw std::invalid_argument if a caller tries to construct with
//   variance ≤ 0.
//
// ── Design questions — answer in comments in kalman1d.cpp ────────────────────
//
// Q1. After 100 predict steps with no updates, what happens to the variance?
//     Does it grow without bound?  What does that mean physically?
//
// Q2. After an update, the fused variance is σ²_fused = σ²_1*σ²_2/(σ²_1+σ²_2).
//     Show algebraically that σ²_fused < σ²_1 and σ²_fused < σ²_2.
//     (Hint: divide numerator and denominator by σ²_2 and simplify.)
//
// Q3. If σ²_1 → ∞ (infinite prior uncertainty), what does the update step
//     reduce to?  (The posterior should equal the measurement.)
//
// Q4. FilterHistory uses a raw pointer and implements Rule of Five.
//     Why not use std::vector<GaussianState>?  What is the pedagogical purpose
//     of requiring raw pointer management here?

#pragma once

#include <string>
#include <ostream>
#include <stdexcept>

namespace bayes {

// ── GaussianState ─────────────────────────────────────────────────────────────
// Represents a 1D Gaussian distribution N(mean, variance).
// Invariant: variance > 0.  The constructor throws std::invalid_argument if
// variance ≤ 0.
//
// operator* implements the Bayesian product (update step fusion):
//   result.mean     = (mean * other.variance + other.mean * variance)
//                     / (variance + other.variance)
//   result.variance = (variance * other.variance)
//                     / (variance + other.variance)
//
// This is mathematically equivalent to the Kalman gain formula for the
// 1D scalar case.  Work it out: with K = σ²_prior / (σ²_prior + σ²_meas),
//   μ_post = μ_prior + K * (z - μ_prior)  →  same result as above.
struct GaussianState {
    double mean;
    double variance;  // invariant: > 0

    // TODO: Constructor — validate variance > 0, throw std::invalid_argument
    // with message "variance must be positive" if violated.
    GaussianState(double mean, double variance);

    // TODO: operator* — Bayesian fusion of two independent Gaussians.
    // THINK: is the result symmetric?  I.e., does (a * b) == (b * a)?
    // THINK: is the result variance less than both inputs?  Prove it.
    GaussianState operator*(const GaussianState& other) const;

    // TODO: operator== — both fields within 1e-9
    bool operator==(const GaussianState& other) const;

    // TODO: operator<< — "N(mean=<m>, var=<v>)" with 4 decimal places
    friend std::ostream& operator<<(std::ostream& os, const GaussianState& g);
};


// ── BayesFilter (abstract base) ───────────────────────────────────────────────
// Abstract interface for a Bayesian filter over a 1D Gaussian state.
//
// predict(control, dt):
//   Advance the state estimate forward in time with the given control input.
//   For a constant-velocity model: mean += control * dt, var += process_noise * dt^2.
//
// update(measurement):
//   Fuse the current belief with a new measurement Gaussian.
//   Use GaussianState::operator* to perform the fusion.
//
// state(): returns the current belief as a GaussianState.
//
// reset(initial): replaces the current belief with a new initial state.
//
// log(label): NON-virtual, defined in the base class.
//   Prints: "<label>: <state>\n"
//   Calls state() (virtual dispatch) to get the current estimate.
class BayesFilter {
public:
    virtual ~BayesFilter() = default;

    virtual void          predict(double control, double dt) = 0;
    virtual void          update(const GaussianState& measurement) = 0;
    virtual GaussianState state() const = 0;
    virtual void          reset(const GaussianState& initial) = 0;

    // TODO: implement log() in kalman1d.cpp
    // Non-virtual: always prints the label and calls state() polymorphically.
    void log(const std::string& label) const;
};


// ── FilterHistory ─────────────────────────────────────────────────────────────
// A circular buffer of GaussianState snapshots using a raw heap pointer.
// Rule of Five required.
//
// Capacity is fixed at construction.  When full, record() overwrites the oldest.
// at(i) returns the i-th oldest entry (0 = oldest), throws std::out_of_range.
//
// This is structurally identical to ScanBuffer from exercise 02, but for
// GaussianState objects instead of float arrays.  GaussianState is a plain
// struct (no heap allocation), so there is only ONE level of heap here.
//
// The Rule of Five is still required because data_ is a raw pointer:
//   - Destructor: delete[] data_
//   - Copy constructor: allocate new data_, copy all GaussianState values
//   - Copy assignment: free old, deep copy (handle self-assignment)
//   - Move constructor: steal data_, null out source
//   - Move assignment: free own, steal, handle self-assignment
class FilterHistory {
public:
    // TODO: constructor — allocate data_ = new GaussianState[capacity], store capacity.
    // Initialise size_ = head_ = 0.
    // TRICKY: GaussianState has a non-default constructor requiring variance > 0.
    // You cannot use new GaussianState[n] without a default constructor.
    // Solution: use operator new[] to allocate raw memory, or provide a default
    // constructor that sets variance = 1.0 (or 0.0 as a sentinel), OR use
    // placement new.  The simplest approach: add a private default constructor
    // to GaussianState that sets mean=0, variance=1 (mark it private, friend FilterHistory).
    // Alternatively, allocate raw bytes and use placement new.
    // Choose one approach and justify your decision in a comment.
    explicit FilterHistory(int capacity);

    ~FilterHistory();
    FilterHistory(const FilterHistory&);
    FilterHistory& operator=(const FilterHistory&);
    FilterHistory(FilterHistory&&) noexcept;
    FilterHistory& operator=(FilterHistory&&) noexcept;

    // TODO: record — circular insert.  Overwrite oldest when full.
    void record(const GaussianState& s);

    // TODO: at — logical index (0=oldest).
    // Circular formula: physical = (head_ - size_ + i + capacity_) % capacity_
    const GaussianState& at(int i) const;

    int size()     const;
    int capacity() const;

private:
    GaussianState* data_;
    int            capacity_;
    int            size_;
    int            head_;
};


// ── ScopedFilterLogger ────────────────────────────────────────────────────────
// RAII wrapper that logs the before- and after-state of a predict/update call.
//
// Usage:
//   {
//       ScopedFilterLogger logger("predict", filter);
//       filter.predict(v, dt);
//   }
//   // destructor fires: prints "  after: <state>"
//
// Constructor:
//   Stores op_, a const reference to the filter, and a COPY of the current
//   state (before_).  Prints "[<op>] before: <before_state>\n".
//   CRITICAL: store before_ as a VALUE (GaussianState copy), not a reference.
//   The filter state will change during the operation.
//
// Destructor:
//   Prints "  after: <after_state>\n"  (calls filter_.state() for current value)
//
// Non-copyable.
class ScopedFilterLogger {
public:
    // TODO: constructor — print before-state, save copy.
    explicit ScopedFilterLogger(const std::string& op, const BayesFilter& filter);

    // TODO: destructor — print after-state.
    ~ScopedFilterLogger();

    ScopedFilterLogger(const ScopedFilterLogger&)            = delete;
    ScopedFilterLogger& operator=(const ScopedFilterLogger&) = delete;

private:
    std::string         op_;
    const BayesFilter&  filter_;  // observing reference — does not own
    GaussianState       before_;  // VALUE copy of state at construction
};


// ── KalmanFilter1D ────────────────────────────────────────────────────────────
// A 1D Kalman filter with constant-velocity process model.
//
// State: belief over position x, represented as N(mean, variance).
//
// predict(v, dt):
//   state_.mean     += v * dt
//   state_.variance += process_noise_ * dt * dt
//   The variance grows: uncertainty accumulates without measurements.
//
// update(z):
//   state_ = state_ * z    (Bayesian fusion using GaussianState::operator*)
//   The variance shrinks: the measurement reduces uncertainty.
//
// THINK about the relationship between predict and update:
//   - predict() encodes the MOTION MODEL (how we think the robot moves)
//   - update()  encodes the SENSOR MODEL (how we think the sensor measures)
//   - The Kalman filter OPTIMALLY balances these two sources of information
//     when both are truly Gaussian with known noise parameters.
class KalmanFilter1D : public BayesFilter {
public:
    // TODO: constructor — store initial state and process_noise.
    // Throw std::invalid_argument if process_noise < 0.
    KalmanFilter1D(const GaussianState& initial, double process_noise);

    void          predict(double control_velocity, double dt) override;
    void          update(const GaussianState& measurement) override;
    GaussianState state() const override;
    void          reset(const GaussianState& initial) override;

private:
    GaussianState state_;
    double        process_noise_;
};

}  // namespace bayes
