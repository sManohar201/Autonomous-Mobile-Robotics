// kalman1d.cpp — Implementation of the bayes namespace
//
// A1: After 100 predict steps with no updates, the variance grows by
//   process_noise * dt^2 each step. It grows without bound (linearly in steps).
//   Physically: the filter loses confidence in its position estimate because
//   the robot has been moving (with noise) but received no sensor corrections.
//
// A2: Prove σ²_fused < σ²_1:
//   σ²_fused = σ²_1 * σ²_2 / (σ²_1 + σ²_2)
//   σ²_fused < σ²_1 iff σ²_2 / (σ²_1 + σ²_2) < 1
//            iff σ²_2 < σ²_1 + σ²_2
//            iff 0 < σ²_1  — true since both variances are positive.
//   Same argument by symmetry for σ²_fused < σ²_2.
//
// A3: As σ²_1 → ∞:
//   μ_fused  = (μ_1 * σ²_2 + μ_2 * σ²_1) / (σ²_1 + σ²_2)
//            → μ_2  (μ_1 * σ²_2/σ²_1 → 0, leading term is μ_2 * σ²_1/σ²_1)
//   σ²_fused = σ²_1 * σ²_2 / (σ²_1 + σ²_2) → σ²_2
//   The posterior equals the measurement exactly — the infinite prior uncertainty
//   means the filter ignores its own belief and trusts the sensor completely.
//
// A4: FilterHistory uses a raw pointer to force the student to implement the full
//   Rule of Five by hand. std::vector would manage memory automatically, hiding
//   the ownership transfer semantics that are central to this module's learning goal.

#include "kalman1d.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace bayes {

// ── GaussianState ─────────────────────────────────────────────────────────────

GaussianState::GaussianState(double mean, double variance)
    : mean(mean), variance(variance)
{
    if (variance <= 0.0) {
        throw std::invalid_argument("variance must be positive");
    }
}

GaussianState GaussianState::operator*(const GaussianState& other) const {
    double denom = variance + other.variance;
    double fused_mean = (mean * other.variance + other.mean * variance) / denom;
    double fused_var  = (variance * other.variance) / denom;
    return GaussianState(fused_mean, fused_var);
}

bool GaussianState::operator==(const GaussianState& other) const {
    return std::abs(mean     - other.mean)     < 1e-9 &&
           std::abs(variance - other.variance) < 1e-9;
}

std::ostream& operator<<(std::ostream& os, const GaussianState& g) {
    os << std::fixed << std::setprecision(4)
       << "N(mean=" << g.mean << ", var=" << g.variance << ")";
    return os;
}


// ── BayesFilter ───────────────────────────────────────────────────────────────

void BayesFilter::log(const std::string& label) const {
    std::cout << label << ": " << state() << "\n";
}


// ── FilterHistory ─────────────────────────────────────────────────────────────
// GaussianState requires variance > 0, so new GaussianState[n] won't compile
// without a default constructor. We allocate raw bytes and use placement new
// to initialise each slot to N(0, 1.0) — a sentinel value that is valid.

static void init_slots(GaussianState* dst, int n) {
    for (int i = 0; i < n; ++i) {
        ::new (&dst[i]) GaussianState(0.0, 1.0);
    }
}

FilterHistory::FilterHistory(int capacity)
    : data_(nullptr), capacity_(capacity), size_(0), head_(0)
{
    if (capacity <= 0) {
        throw std::invalid_argument("FilterHistory capacity must be positive");
    }
    data_ = static_cast<GaussianState*>(
        ::operator new[](capacity_ * sizeof(GaussianState)));
    init_slots(data_, capacity_);
}

FilterHistory::~FilterHistory() {
    if (data_) {
        for (int i = 0; i < capacity_; ++i) {
            data_[i].~GaussianState();
        }
        ::operator delete[](data_);
    }
}

FilterHistory::FilterHistory(const FilterHistory& other)
    : data_(nullptr), capacity_(other.capacity_), size_(other.size_), head_(other.head_)
{
    data_ = static_cast<GaussianState*>(
        ::operator new[](capacity_ * sizeof(GaussianState)));
    for (int i = 0; i < capacity_; ++i) {
        ::new (&data_[i]) GaussianState(other.data_[i]);
    }
}

FilterHistory& FilterHistory::operator=(const FilterHistory& other) {
    if (this == &other) return *this;
    for (int i = 0; i < capacity_; ++i) data_[i].~GaussianState();
    ::operator delete[](data_);

    capacity_ = other.capacity_;
    size_     = other.size_;
    head_     = other.head_;
    data_ = static_cast<GaussianState*>(
        ::operator new[](capacity_ * sizeof(GaussianState)));
    for (int i = 0; i < capacity_; ++i) {
        ::new (&data_[i]) GaussianState(other.data_[i]);
    }
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
    for (int i = 0; i < capacity_; ++i) data_[i].~GaussianState();
    ::operator delete[](data_);

    data_     = other.data_;
    capacity_ = other.capacity_;
    size_     = other.size_;
    head_     = other.head_;

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
        throw std::out_of_range("FilterHistory::at: index out of range");
    }
    return data_[(head_ - size_ + i + capacity_) % capacity_];
}

int FilterHistory::size()     const { return size_; }
int FilterHistory::capacity() const { return capacity_; }


// ── ScopedFilterLogger ────────────────────────────────────────────────────────

ScopedFilterLogger::ScopedFilterLogger(const std::string& op, const BayesFilter& filter)
    : op_(op), filter_(filter), before_(filter.state())
{
    std::cout << "[" << op_ << "] before: " << before_ << "\n";
}

ScopedFilterLogger::~ScopedFilterLogger() {
    std::cout << "  after: " << filter_.state() << "\n";
}


// ── KalmanFilter1D ────────────────────────────────────────────────────────────

KalmanFilter1D::KalmanFilter1D(const GaussianState& initial, double process_noise)
    : state_(initial), process_noise_(process_noise)
{
    if (process_noise < 0.0) {
        throw std::invalid_argument("process_noise must be non-negative");
    }
}

void KalmanFilter1D::predict(double control_velocity, double dt) {
    state_.mean     += control_velocity * dt;
    state_.variance += process_noise_   * dt * dt;
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
