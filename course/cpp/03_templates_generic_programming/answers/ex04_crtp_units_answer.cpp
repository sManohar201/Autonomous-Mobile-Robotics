#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

template <typename Derived>
class Printable {
public:
    std::string describe() const {
        return static_cast<const Derived&>(*this).describe_impl();
    }
};

class Pose2d : public Printable<Pose2d> {
public:
    Pose2d(double x, double y, double theta)
        : x_{x}, y_{y}, theta_{theta}
    {}

    std::string describe_impl() const {
        (void)x_;
        (void)y_;
        (void)theta_;
        return "Pose2d";
    }

private:
    double x_;
    double y_;
    double theta_;
};

template <typename Tag>
class Quantity {
public:
    explicit constexpr Quantity(double value) : value_{value} {}

    constexpr double value() const { return value_; }

    constexpr Quantity& operator+=(Quantity rhs) {
        value_ += rhs.value_;
        return *this;
    }

private:
    double value_{};
};

template <typename Tag>
constexpr Quantity<Tag> operator+(Quantity<Tag> lhs, Quantity<Tag> rhs) {
    lhs += rhs;
    return lhs;
}

struct MeterTag {};
struct SecondTag {};

using Meters = Quantity<MeterTag>;
using Seconds = Quantity<SecondTag>;

double speed(Meters distance, Seconds time) {
    if (time.value() <= 0.0) {
        throw std::invalid_argument("time must be positive");
    }
    return distance.value() / time.value();
}

int main() {
    Pose2d p{1.0, 2.0, 0.5};
    assert(p.describe() == "Pose2d");

    Meters a{3.0};
    Meters b{4.0};
    auto c = a + b;
    assert(c.value() == 7.0);

    Seconds t{2.0};
    assert(std::abs(speed(c, t) - 3.5) < 1e-9);

    std::cout << "ex04_crtp_units_answer passed\n";
}

