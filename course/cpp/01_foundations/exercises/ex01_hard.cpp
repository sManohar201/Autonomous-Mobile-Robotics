// Exercise 01 (Hard) — One Definition Rule, Linkage, Inline, and Symbol Visibility
// ─────────────────────────────────────────────────────────────────────────────
//
// CONTEXT:
//   In large robotics codebases, sensor utility functions are shared across
//   50+ translation units. "Multiple definition" linker errors and a "global"
//   counter that silently gives different values in different modules are both
//   ODR and linkage bugs. This exercise drills the rules cold.
//
// ─────────────────────────────────────────────────────────────────────────────
// PRE-CODING RESEARCH QUESTIONS — Answer each question in the space provided
// BEFORE writing any code. These answers are part of your grade.
// ─────────────────────────────────────────────────────────────────────────────

// Q1. The One Definition Rule has two parts:
//     (a) at most one definition per translation unit (TU)
//     (b) exactly one definition in the entire program for some entities
//
//     Which entities may have MORE THAN ONE definition across TUs
//     (provided all definitions are token-for-token identical)?
//       (a) non-inline function
//       (b) inline function
//       (c) class definition
//       (d) constexpr function
//       (e) constexpr variable (C++17)
//       (f) non-const global variable
//
//     YOUR ANSWER:
//     TODO

// Q2. A non-inline function is defined (with body) in a .hpp header, included
//     by sensor.cpp AND main.cpp. Trace the build:
//     (a) What does the compiler do with sensor.cpp?
//     (b) What does the compiler do with main.cpp?
//     (c) What does the linker see?
//     (d) What is the exact error, and at which build stage does it occur?
//
//     YOUR ANSWER:
//     TODO

// Q3. Linkage:
//     (a) What is external linkage? Give two examples.
//     (b) What is internal linkage? Give two ways to achieve it.
//     (c) What has no linkage? Give one example.
//
//     YOUR ANSWER:
//     TODO

// Q4. Research the `nm` command. After compiling an object file, what do these
//     symbol type codes mean: T, U, W?
//     If you put clamp_range in an anonymous namespace, what does nm show?
//
//     YOUR ANSWER:
//     TODO

// Q5. extern "C":
//     (a) Why does C++ mangle function names?
//     (b) What does extern "C" disable, and why is this needed when a C++
//         library must be called from a C-based embedded ROS driver?
//
//     YOUR ANSWER:
//     TODO

// Q6. A header contains `static int call_count = 0;`. This compiles.
//     (a) What does `static` mean at file scope?
//     (b) How many copies of call_count exist if the header is included from
//         3 .cpp files?
//     (c) Why is this usually a bug? What is the correct pattern for a shared
//         counter?
//
//     YOUR ANSWER:
//     TODO

// ─────────────────────────────────────────────────────────────────────────────
// PART A — Bug Hunt
//
// For each header pseudocode snippet below, diagnose:
//   1. Is there a bug?
//   2. If so, which rule is violated and what is the exact error?
//   3. At which build stage does the error occur?
//   4. How would you fix it?
//
// Do NOT write code for Part A — answer in comments only.
// ─────────────────────────────────────────────────────────────────────────────

// A1 — utils.hpp (included by sensor.cpp AND main.cpp):
//
//   #pragma once
//   #include <cmath>
//   double wrap_angle(double r) { return std::fmod(r, 6.28318); }
//   constexpr double MAX_RANGE = 30.0;
//
// YOUR DIAGNOSIS:
// TODO

// A2 — config.hpp (included by node.cpp AND logger.cpp):
//
//   #pragma once
//   static int dropped_packets = 0;
//
// YOUR DIAGNOSIS:
// TODO

// A3 — shared.hpp (included by main.cpp AND worker.cpp):
//
//   #pragma once
//   int total_frames = 0;
//
// YOUR DIAGNOSIS:
// TODO

// A4 — math_utils.hpp (included from 5 .cpp files):
//
//   #pragma once
//   namespace { double sq(double x) { return x * x; } }
//
// YOUR DIAGNOSIS:
// TODO

// ─────────────────────────────────────────────────────────────────────────────
// PART B — Implement Angle Utilities With Correct Linkage
// ─────────────────────────────────────────────────────────────────────────────

#include <iostream>
#include <iomanip>
#include <cmath>

constexpr double PI = 3.14159265358979323846;

// B1. constexpr double deg_to_rad(double degrees)
//     Must be evaluatable at compile time.
//     THINK: Why doesn't putting this in a header violate ODR?
//            (Answer in comment before implementation)
//
// TODO: YOUR ANSWER:
//
// TODO: Implement deg_to_rad
constexpr double deg_to_rad(double degrees) {
    return 0.0; // TODO: replace this stub
}

// B2. inline double wrap_to_pi(double rad)
//     Wrap angle into (-π, π].
//     Algorithm:
//       1. shift by π:        r = rad + π
//       2. fmod by 2π:        r = std::fmod(r, 2π)
//       3. fix negatives:     if r < 0, r += 2π
//       4. shift back:        return r - π
//     THINK: Why must it be inline if it lives in a header included by many TUs?
//            (Answer in comment before implementation)
//
// TODO: YOUR ANSWER:
//
// TODO: Implement wrap_to_pi
inline double wrap_to_pi(double rad) {
    return 0.0; // TODO: replace this stub
}

// B3. clamp_range with INTERNAL LINKAGE
//     Clamp r to [0.0, max_r]. Must NOT be visible outside this TU.
//     THINK: How is anonymous namespace different from `static` at file scope
//            for a function? Are there cases where they behave differently?
//            (Answer in comment before implementation)
//
// TODO: YOUR ANSWER:
//
// TODO: Implement clamp_range inside an anonymous namespace below.
namespace {
    double clamp_range(double r, double max_r) {
        return 0.0; // TODO: replace this stub
    }
}

// B4. AngleAccumulator class
//     THINK: Where exactly must instance_count_ be defined?
//            What ODR clause governs static data members?
//            (Answer in comment before implementation)
//
// TODO: YOUR ANSWER:
//
// TODO: Implement the class below. Then define instance_count_ at namespace
//       scope below the class.

class AngleAccumulator {
public:
    // TODO: explicit constructor(double initial = 0.0)
    //       Wrap initial using wrap_to_pi, store in sum_, increment instance_count_
    explicit AngleAccumulator(double initial = 0.0) : sum_(0.0) {
        (void)initial; // TODO: replace stub — wrap initial and increment count
    }

    // TODO: void add(double rad)
    //       Wrap rad using wrap_to_pi, add result to sum_
    void add(double rad) {
        (void)rad; // TODO: replace stub
    }

    // TODO: double total() const
    double total() const {
        return sum_; // TODO: replace stub
    }

    // TODO: static int active()
    static int active() {
        return instance_count_; // TODO: replace stub
    }

    // TODO: destructor — decrement instance_count_
    ~AngleAccumulator() {
        // TODO: decrement instance_count_
    }

private:
    double sum_ = 0.0;
    static int instance_count_;
};

// TODO: Define instance_count_ here (outside the class body).
int AngleAccumulator::instance_count_ = 0;

// ─────────────────────────────────────────────────────────────────────────────
// TOOL EXERCISE (no submission required):
//   After compiling this file, run:
//     g++ -c ex01_hard.cpp -o ex01_hard.o && nm -C ex01_hard.o
//   Observe:
//   - Does clamp_range appear? Under what mangled name?
//   - What symbol type letters appear for deg_to_rad and wrap_to_pi?
//   - What does the W symbol type mean for inline functions?
// ─────────────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────
// EXPECTED OUTPUT:
//   deg_to_rad(90)  = 1.57080
//   deg_to_rad(180) = 3.14159
//   wrap_to_pi(7.0) = 0.71681
//   wrap_to_pi(-7.0) = -0.71681
//   clamp_range(35.5, 30.0) = 30.00000
//   clamp_range(-1.5, 30.0) = 0.00000
//   total after additions = 3.00000
//   active: 2
//   active after scope: 0
// ─────────────────────────────────────────────────────────────────────────────

int main() {
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "deg_to_rad(90)  = " << deg_to_rad(90.0)  << "\n";
    std::cout << "deg_to_rad(180) = " << deg_to_rad(180.0) << "\n";
    std::cout << "wrap_to_pi(7.0) = "  << wrap_to_pi(7.0)   << "\n";
    std::cout << "wrap_to_pi(-7.0) = " << wrap_to_pi(-7.0)  << "\n";
    std::cout << "clamp_range(35.5, 30.0) = " << clamp_range(35.5, 30.0) << "\n";
    std::cout << "clamp_range(-1.5, 30.0) = " << clamp_range(-1.5, 30.0) << "\n";
    {
        AngleAccumulator a1;
        AngleAccumulator a2;
        a1.add(1.5);
        a1.add(-0.5);
        a1.add(2.0);
        std::cout << "total after additions = " << a1.total() << "\n";
        std::cout << "active: " << AngleAccumulator::active() << "\n";
    }
    std::cout << "active after scope: " << AngleAccumulator::active() << "\n";
    return 0;
}
