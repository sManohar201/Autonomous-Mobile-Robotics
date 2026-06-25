// Exercise 04 (Hard) - CRTP Static Filter Interface
//
// Context:
//   Some filters are selected at compile time and should avoid virtual calls.
//
// Tasks:
//   1. Implement FilterBase<Derived> with predict/update/state forwarding.
//   2. Implement ConstantVelocityFilter.
//   3. Implement run_filter(FilterBase<Derived>&, dt, measurement).
//   4. Explain why this cannot replace runtime plugin polymorphism.
//
// Q4 answer — CRTP vs runtime polymorphism:
//   CRTP resolves method calls at compile time using static dispatch. The
//   filter type must be known at compile time — you cannot store a
//   FilterBase<X> and a FilterBase<Y> in the same container (they are
//   different types) or choose between them based on a runtime condition
//   (e.g., a config file). Runtime polymorphism (virtual) allows the type
//   to be selected at runtime and heterogeneous containers. The two patterns
//   are complementary: CRTP for performance-critical inner loops where the
//   type is known; virtual for plugin systems, factory patterns, or when
//   the filter type comes from user config.

#include <cassert>
#include <iostream>

template <typename Derived>
class FilterBase {
public:
    void   predict(double dt)     { self().predict_impl(dt); }
    void   update(double z)       { self().update_impl(z); }
    double state() const          { return self().state_impl(); }

private:
    Derived&       self()       { return static_cast<Derived&>(*this); }
    const Derived& self() const { return static_cast<const Derived&>(*this); }
};

class ConstantVelocityFilter : public FilterBase<ConstantVelocityFilter> {
public:
    explicit ConstantVelocityFilter(double velocity) : velocity_(velocity) {}

    void   predict_impl(double dt) { x_ += velocity_ * dt; }
    void   update_impl(double z)   { x_ = 0.5 * x_ + 0.5 * z; }
    double state_impl() const      { return x_; }

private:
    double x_{0.0};
    double velocity_;
};

template <typename Filter>
void run_filter(FilterBase<Filter>& filter, double dt, double measurement) {
    filter.predict(dt);
    filter.update(measurement);
}

int main() {
    ConstantVelocityFilter f{2.0};
    run_filter(f, 3.0, 10.0);
    // predict: x = 0 + 2*3 = 6; update: x = 0.5*6 + 0.5*10 = 8
    assert(f.state() == 8.0);

    std::cout << "ex04_hard passed\n";
}

