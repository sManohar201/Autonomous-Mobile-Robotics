#include <cassert>
#include <iostream>

template <typename Derived>
class FilterBase {
public:
    void predict(double dt) { self().predict_impl(dt); }
    void update(double z) { self().update_impl(z); }
    double state() const { return self().state_impl(); }

private:
    Derived& self() { return static_cast<Derived&>(*this); }
    const Derived& self() const { return static_cast<const Derived&>(*this); }
};

class ConstantVelocityFilter : public FilterBase<ConstantVelocityFilter> {
public:
    explicit ConstantVelocityFilter(double velocity) : velocity_{velocity} {}
    void predict_impl(double dt) { x_ += velocity_ * dt; }
    void update_impl(double z) { x_ = 0.5 * x_ + 0.5 * z; }
    double state_impl() const { return x_; }

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
    assert(f.state() == 8.0);
    std::cout << "ex04_hard_answer passed\n";
}

