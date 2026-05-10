#include <cassert>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

class Localizer {
public:
    virtual ~Localizer() = default;
    virtual void update(double z) = 0;
    virtual double state() const = 0;
};

class DirectLocalizer : public Localizer {
public:
    void update(double z) override { x_ = z; }
    double state() const override { return x_; }
private:
    double x_{0.0};
};

class BlendLocalizer : public Localizer {
public:
    void update(double z) override { x_ = 0.5 * x_ + 0.5 * z; }
    double state() const override { return x_; }
private:
    double x_{0.0};
};

struct FactoryResult {
    std::unique_ptr<Localizer> plugin;
    std::string error;
};

FactoryResult create_localizer(const std::string& type) {
    if (type == "direct") return {std::make_unique<DirectLocalizer>(), ""};
    if (type == "blend") return {std::make_unique<BlendLocalizer>(), ""};
    return {nullptr, "unknown localizer: " + type};
}

int main() {
    auto result = create_localizer("blend");
    assert(result.plugin);
    result.plugin->update(10.0);
    assert(result.plugin->state() == 5.0);
    assert(!create_localizer("bad").plugin);
    std::cout << "ex02_hard_answer passed\n";
}

