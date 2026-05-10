#include <cassert>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

class Localizer {
public:
    virtual ~Localizer() = default;
    virtual void predict(double dt) = 0;
    virtual void update(double measurement) = 0;
    virtual double state() const = 0;
};

class DeadReckoning : public Localizer {
public:
    void predict(double dt) override { x_ += velocity_ * dt; }
    void update(double measurement) override { velocity_ = measurement; }
    double state() const override { return x_; }

private:
    double x_{0.0};
    double velocity_{0.0};
};

class ConstantLocalizer : public Localizer {
public:
    void predict(double) override {}
    void update(double measurement) override { x_ = measurement; }
    double state() const override { return x_; }

private:
    double x_{0.0};
};

std::unique_ptr<Localizer> create_localizer(const std::string& name) {
    if (name == "dead_reckoning") return std::make_unique<DeadReckoning>();
    if (name == "constant") return std::make_unique<ConstantLocalizer>();
    throw std::invalid_argument("unknown localizer");
}

int main() {
    auto loc = create_localizer("dead_reckoning");
    loc->update(2.0);
    loc->predict(3.0);
    assert(loc->state() == 6.0);
    std::cout << "ex02_plugin_factory_answer passed\n";
}

