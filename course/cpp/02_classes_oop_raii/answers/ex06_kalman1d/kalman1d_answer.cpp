// kalman1d_answer.cpp — 1D Kalman Filter implementation
// Module 2 Capstone — answer file

// ── Design questions ─────────────────────────────────────────────────────────
//
// Q1. After 100 predict steps (no updates), what happens to variance?
//     Formula: var_k = var_0 + k * q * dt²
//     With q=0.1, dt=0.1: each step adds 0.001.  After 100 steps: +0.1.
//     Variance grows without bound (linearly in the number of steps).
//     Physically: the robot is moving with an imperfect model.  Without sensor
//     updates to constrain our belief, uncertainty accumulates indefinitely —
//     we become less and less sure of where the robot actually is.
//
// Q2. Show σ²_fused < σ²_1 and σ²_fused < σ²_2.
//     σ²_fused = (σ²_1 · σ²_2) / (σ²_1 + σ²_2)
//     Divide numerator and denominator by σ²_2:
//       σ²_fused = σ²_1 / (σ²_1/σ²_2 + 1)
//     Since σ²_1/σ²_2 + 1 > 1, we have σ²_fused < σ²_1.
//     By symmetry (swap indices), σ²_fused < σ²_2.
//     QED.  Sensor fusion always REDUCES uncertainty.
//
// Q3. If σ²_1 → ∞ (infinite prior uncertainty):
//     μ_fused = (μ_1·σ²_2 + μ_2·σ²_1) / (σ²_1 + σ²_2)
//             → (μ_2·σ²_1) / σ²_1   as σ²_1 → ∞
//             = μ_2
//     σ²_fused = (σ²_1·σ²_2) / (σ²_1 + σ²_2)
//              → σ²_2   as σ²_1 → ∞
//     Result: the posterior equals the measurement N(μ_2, σ²_2).
//     Intuition: if we know nothing about the prior, we trust the measurement
//     completely — the filter initialises itself from the first observation.
//
// Q4. FilterHistory uses a raw pointer (Rule of Five) instead of std::vector.
//     A std::vector<GaussianState> would handle memory automatically and make
//     Rule of Five unnecessary.  The pedagogical purpose is to practice:
//       - heap allocation / deallocation ownership rules
//       - deep copy vs shallow copy (copy ctor / copy assignment)
//       - move semantics: stealing the pointer and nulling the source
//       - self-assignment guards in copy and move assignment
//     These are the exact same skills required for any resource-owning class
//     (file handles, GPU buffers, network connections) where std::vector is not
//     available or appropriate.  Rule of Five fluency is essential for
//     understanding what the standard library containers do internally.

#include "kalman1d.hpp"
#include <algorithm>   // std::copy
#include <cmath>       // std::abs
#include <iomanip>
#include <iostream>

namespace bayes {

// ── GaussianState ─────────────────────────────────────────────────────────────

GaussianState::GaussianState(double m, double v)
    : mean(m), variance(v)
{
    if (variance <= 0.0) {
        throw std::invalid_argument("variance must be positive");
    }
}

GaussianState GaussianState::operator*(const GaussianState& other) const {
    double new_mean = (mean * other.variance + other.mean * variance)
                      / (variance + other.variance);
    double new_var  = (variance * other.variance)
                      / (variance + other.variance);
    return GaussianState(new_mean, new_var);
}

bool GaussianState::operator==(const GaussianState& other) const {
    return std::abs(mean     - other.mean)     < 1e-9
        && std::abs(variance - other.variance) < 1e-9;
}

std::ostream& operator<<(std::ostream& os, const GaussianState& g) {
    os << "N(mean=" << std::fixed << std::setprecision(4)
       << g.mean << ", var=" << g.variance << ")";
    return os;
}


// ── BayesFilter ───────────────────────────────────────────────────────────────

void BayesFilter::log(const std::string& label) const {
    std::cout << label << ": " << state() << "\n";
}


// ── FilterHistory ─────────────────────────────────────────────────────────────
// Uses the private GaussianState() default constructor (friend declaration in
// the header) so that `new GaussianState[capacity]` compiles.  Each slot is
// overwritten by record() before it is ever read via at(), so the default
// initial values (mean=0, var=1) are never visible to the caller.

FilterHistory::FilterHistory(int capacity)
    : capacity_(capacity), size_(0), head_(0)
{
    data_ = new GaussianState[capacity_];
}

FilterHistory::~FilterHistory() {
    delete[] data_;
}

FilterHistory::FilterHistory(const FilterHistory& other)
    : capacity_(other.capacity_), size_(other.size_), head_(other.head_)
{
    data_ = new GaussianState[capacity_];
    std::copy(other.data_, other.data_ + capacity_, data_);
}

FilterHistory& FilterHistory::operator=(const FilterHistory& other) {
    if (this == &other) return *this;
    delete[] data_;
    capacity_ = other.capacity_;
    size_     = other.size_;
    head_     = other.head_;
    data_     = new GaussianState[capacity_];
    std::copy(other.data_, other.data_ + capacity_, data_);
    return *this;
}

FilterHistory::FilterHistory(FilterHistory&& other) noexcept
    : data_(other.data_), capacity_(other.capacity_),
      size_(other.size_), head_(other.head_)
{
    other.data_     = nullptr;
    other.capacity_ = 0;
    other.size_     = 0;
    other.head_     = 0;
}

FilterHistory& FilterHistory::operator=(FilterHistory&& other) noexcept {
    if (this == &other) return *this;
    delete[] data_;
    data_           = other.data_;
    capacity_       = other.capacity_;
    size_           = other.size_;
    head_           = other.head_;
    other.data_     = nullptr;
    other.capacity_ = 0;
    other.size_     = 0;
    other.head_     = 0;
    return *this;
}

void FilterHistory::record(const GaussianState& s) {
    data_[head_] = s;
    head_ = (head_ + 1) % capacity_;
    if (size_ < capacity_) ++size_;
}

const GaussianState& FilterHistory::at(int i) const {
    if (i < 0 || i >= size_) {
        throw std::out_of_range("FilterHistory::at index out of range");
    }
    int physical = (head_ - size_ + i + capacity_) % capacity_;
    return data_[physical];
}

int FilterHistory::size()     const { return size_; }
int FilterHistory::capacity() const { return capacity_; }


// ── ScopedFilterLogger ────────────────────────────────────────────────────────

ScopedFilterLogger::ScopedFilterLogger(const std::string& op,
                                       const BayesFilter& filter)
    : op_(op), filter_(filter), before_(filter.state())
{
    std::cout << "[" << op_ << "] before: " << before_ << "\n";
}

ScopedFilterLogger::~ScopedFilterLogger() {
    std::cout << "  after: " << filter_.state() << "\n";
}


// ── KalmanFilter1D ────────────────────────────────────────────────────────────

KalmanFilter1D::KalmanFilter1D(const GaussianState& initial,
                               double process_noise)
    : state_(initial), process_noise_(process_noise)
{
    if (process_noise < 0.0) {
        throw std::invalid_argument("process noise must be non-negative");
    }
}

void KalmanFilter1D::predict(double control_velocity, double dt) {
    state_.mean     += control_velocity * dt;
    state_.variance += process_noise_ * dt * dt;
}

void KalmanFilter1D::update(const GaussianState& measurement) {
    state_ = state_ * measurement;
}

GaussianState KalmanFilter1D::state() const {
    return state_;
}

void KalmanFilter1D::reset(const GaussianState& initial) {
    state_ = initial;
}

}  // namespace bayes
