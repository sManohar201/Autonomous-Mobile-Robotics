// Exercise 04 - CRTP and Strongly Typed Units
//
// Goal:
//   Use templates to remove virtual dispatch and prevent unit mixups.

#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

// TODO 1: Implement Printable<Derived> using CRTP.
// It should provide std::string describe() const that calls
// static_cast<const Derived&>(*this).describe_impl().

class Pose2d /* TODO: inherit Printable<Pose2d> */ {
public:
    Pose2d(double x, double y, double theta)
        : x_{x}, y_{y}, theta_{theta}
    {}

    // TODO 2: implement describe_impl().
    // Format can be simple, e.g. "Pose2d".

private:
    double x_;
    double y_;
    double theta_;
};

template <typename Tag>
class Quantity {
public:
    // TODO 3: explicit constexpr constructor from double.

    // TODO 4: constexpr value() accessor.

    // TODO 5: operator+= only for the same Quantity<Tag>.

private:
    double value_{};
};

// TODO 6: operator+ for same-unit quantities.

struct MeterTag {};
struct SecondTag {};

using Meters = Quantity<MeterTag>;
using Seconds = Quantity<SecondTag>;

// TODO 7: Implement speed(Meters distance, Seconds time), returning double.
// Throw std::invalid_argument if time.value() <= 0.

int main() {
    // TODO: make these pass.
    // Pose2d p{1.0, 2.0, 0.5};
    // assert(p.describe() == "Pose2d");
    //
    // Meters a{3.0};
    // Meters b{4.0};
    // auto c = a + b;
    // assert(c.value() == 7.0);
    //
    // Seconds t{2.0};
    // assert(std::abs(speed(c, t) - 3.5) < 1e-9);
    //
    // This should not compile, and that is the point:
    // auto invalid = a + t;

    std::cout << "ex04_crtp_units passed\n";
}

