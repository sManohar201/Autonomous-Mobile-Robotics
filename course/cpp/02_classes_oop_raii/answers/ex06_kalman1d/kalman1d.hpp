// kalman1d.hpp — answer version
// Identical to the exercise header, with one addition:
//   GaussianState gains a private default constructor (mean=0, variance=1)
//   so FilterHistory can call `new GaussianState[capacity]` without
//   resorting to raw-byte allocation.  The private access + friend keeps
//   the invariant: no public code can create an invalid GaussianState.

#pragma once

#include <string>
#include <ostream>
#include <stdexcept>

namespace bayes {

struct GaussianState {
    double mean;
    double variance;  // invariant: > 0

    GaussianState(double mean, double variance);

    GaussianState operator*(const GaussianState& other) const;
    bool operator==(const GaussianState& other) const;
    friend std::ostream& operator<<(std::ostream& os, const GaussianState& g);

private:
    // Private default constructor used exclusively by FilterHistory to fill
    // the raw array.  Sets variance=1 so the invariant (var>0) holds even
    // for uninitialised slots.
    GaussianState() : mean(0.0), variance(1.0) {}
    friend class FilterHistory;
};


class BayesFilter {
public:
    virtual ~BayesFilter() = default;

    virtual void          predict(double control, double dt) = 0;
    virtual void          update(const GaussianState& measurement) = 0;
    virtual GaussianState state() const = 0;
    virtual void          reset(const GaussianState& initial) = 0;

    void log(const std::string& label) const;
};


class FilterHistory {
public:
    explicit FilterHistory(int capacity);
    ~FilterHistory();
    FilterHistory(const FilterHistory&);
    FilterHistory& operator=(const FilterHistory&);
    FilterHistory(FilterHistory&&) noexcept;
    FilterHistory& operator=(FilterHistory&&) noexcept;

    void record(const GaussianState& s);
    const GaussianState& at(int i) const;

    int size()     const;
    int capacity() const;

private:
    GaussianState* data_;
    int            capacity_;
    int            size_;
    int            head_;
};


class ScopedFilterLogger {
public:
    explicit ScopedFilterLogger(const std::string& op, const BayesFilter& filter);
    ~ScopedFilterLogger();

    ScopedFilterLogger(const ScopedFilterLogger&)            = delete;
    ScopedFilterLogger& operator=(const ScopedFilterLogger&) = delete;

private:
    std::string        op_;
    const BayesFilter& filter_;
    GaussianState      before_;
};


class KalmanFilter1D : public BayesFilter {
public:
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
