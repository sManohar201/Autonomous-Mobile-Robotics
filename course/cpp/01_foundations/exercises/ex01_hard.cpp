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
//     (b), (c), (d), (e) may appear in multiple TUs identically.
//     inline functions: one definition per TU is allowed (identical bodies required).
//     class definitions: #include brings the same definition into many TUs — fine.
//     constexpr functions: implicitly inline, same rule applies.
//     constexpr variables (C++17): implicitly inline, one per TU is OK.
//     (a) and (f) must have exactly ONE definition across the entire program.

// Q2. A non-inline function is defined (with body) in a .hpp header, included
//     by sensor.cpp AND main.cpp. Trace the build:
//     (a) What does the compiler do with sensor.cpp?
//         → Compiles sensor.cpp; the #include pastes the function body into
//           sensor.cpp's translation unit. Compiler emits sensor.o with the symbol.
//     (b) What does the compiler do with main.cpp?
//         → Same: compiles main.cpp, pastes the body, emits main.o with the same symbol.
//     (c) What does the linker see?
//         → Two object files both exporting the same non-inline symbol.
//     (d) What is the exact error, and at which build stage does it occur?
//         → Linker error: "multiple definition of `func_name'". Occurs at link time.

// Q3. Linkage:
//     (a) External linkage: the name is visible to the entire program (other TUs).
//         Examples: non-static functions, non-static global variables.
//     (b) Internal linkage: the name is local to the TU.
//         Two ways: `static` at file scope, or anonymous namespace.
//     (c) No linkage: local variables. Example: `void f() { int x = 0; }` — x has no linkage.

// Q4. nm symbol type codes:
//     T — symbol is in the text (code) section, externally visible (external linkage).
//     U — undefined; the TU references this symbol but doesn't define it.
//     W — weak symbol (e.g. inline function body). Linker picks one definition if multiple exist.
//
//     If clamp_range is in an anonymous namespace:
//     nm shows it with a mangled name like _ZN12_GLOBAL__N_111clamp_rangeEdd
//     and type 't' (lowercase = internal/local linkage).

// Q5. extern "C":
//     (a) C++ mangles function names to encode parameter types, enabling overloading.
//         E.g. void f(int) → _Z1fi, void f(double) → _Z1fd.
//     (b) extern "C" disables name mangling for the declared function.
//         Needed when a C++ library exports a symbol that must be called from a
//         C-based embedded ROS driver: the C linker expects the plain unmangled
//         name, not the C++ mangled form.

// Q6. `static int call_count = 0` in a header:
//     (a) `static` at file scope gives the variable internal linkage (TU-local).
//     (b) 3 copies exist — one per TU that includes the header.
//     (c) Each TU gets its own independent counter. Incrementing from sensor.cpp
//         doesn't affect main.cpp's counter. This is silently wrong.
//         Correct pattern: declare `extern int call_count;` in the header,
//         define `int call_count = 0;` in exactly one .cpp file.

// ─────────────────────────────────────────────────────────────────────────────
// PART A — Bug Hunt
// ─────────────────────────────────────────────────────────────────────────────

// A1 — utils.hpp (included by sensor.cpp AND main.cpp):
//   #pragma once
//   #include <cmath>
//   double wrap_angle(double r) { return std::fmod(r, 6.28318); }
//   constexpr double MAX_RANGE = 30.0;
//
// DIAGNOSIS:
//   BUG: wrap_angle has a non-inline body in a header. Each TU that includes it
//   gets a separate definition. Linker error: "multiple definition of wrap_angle".
//   FIX: declare `inline double wrap_angle(...)` or move the body to a .cpp file.
//   MAX_RANGE: constexpr at namespace scope is implicitly inline (C++17) — fine.

// A2 — config.hpp (included by node.cpp AND logger.cpp):
//   #pragma once
//   static int dropped_packets = 0;
//
// DIAGNOSIS:
//   No linker error (static gives internal linkage). BUT each TU gets its own
//   independent dropped_packets. Incrementing in node.cpp is invisible to logger.cpp.
//   This is a silent logic bug. FIX: extern int dropped_packets; + one definition.

// A3 — shared.hpp (included by main.cpp AND worker.cpp):
//   #pragma once
//   int total_frames = 0;
//
// DIAGNOSIS:
//   BUG: non-const, non-static, non-inline global variable defined in a header.
//   Two TUs both define it → linker error: "multiple definition of total_frames".
//   FIX: `extern int total_frames;` in header, `int total_frames = 0;` in one .cpp.

// A4 — math_utils.hpp (included from 5 .cpp files):
//   #pragma once
//   namespace { double sq(double x) { return x * x; } }
//
// DIAGNOSIS:
//   Not a bug. Anonymous namespace gives sq internal linkage in each TU.
//   5 separate sq symbols, each with internal linkage — no ODR violation.
//   The linker never sees them as conflicting definitions.
//   Potential issue: binary bloat (5 copies of sq exist in the final binary).

// ─────────────────────────────────────────────────────────────────────────────
// PART B — Implement Angle Utilities With Correct Linkage
// ─────────────────────────────────────────────────────────────────────────────

#include <iostream>
#include <iomanip>
#include <cmath>

constexpr double PI = 3.14159265358979323846;

// B1. constexpr double deg_to_rad(double degrees)
//     WHY it doesn't violate ODR when in a header:
//     constexpr functions are implicitly inline. Inline functions may appear
//     in multiple TUs (their definitions must be identical). The linker
//     selects one copy. This is explicitly permitted by the ODR.
//
constexpr double deg_to_rad(double degrees) {
    return degrees * (PI / 180.0);
}

// B2. inline double wrap_to_pi(double rad)
//     WHY it must be inline if it lives in a header included by many TUs:
//     Without inline, each TU that includes the header defines the function body.
//     The linker sees multiple definitions → error. `inline` tells the linker
//     "these identical copies are all the same definition; pick one."
//
inline double wrap_to_pi(double rad) {
    double r = rad + PI;
    r = std::fmod(r, 2.0 * PI);
    if (r < 0.0) r += 2.0 * PI;
    return r - PI;
}

// B3. clamp_range with INTERNAL LINKAGE via anonymous namespace.
//     anonymous namespace vs static:
//     Both give internal linkage for functions/variables. The key difference:
//     anonymous namespace also gives internal linkage to type definitions (structs,
//     classes) and enums — `static` cannot do that. For functions and variables
//     they are equivalent in practice. Prefer anonymous namespace in C++ (more general).
//
namespace {
    double clamp_range(double r, double max_r) {
        if (r < 0.0)    return 0.0;
        if (r > max_r)  return max_r;
        return r;
    }
}

// B4. AngleAccumulator class
//     WHERE instance_count_ must be defined:
//     Static data members are declared inside the class but must be DEFINED at
//     namespace scope exactly once across the entire program (in one .cpp file).
//     The ODR governs static data members the same as non-inline non-const globals.
//     (C++17 inline static data members relax this — they can be defined in the class
//     with `inline static int x = 0;` — but the traditional pattern is one .cpp definition.)

class AngleAccumulator {
public:
    explicit AngleAccumulator(double initial = 0.0) : sum_(wrap_to_pi(initial)) {
        ++instance_count_;
    }

    void add(double rad) {
        sum_ += wrap_to_pi(rad);
    }

    double total() const {
        return sum_;
    }

    static int active() {
        return instance_count_;
    }

    ~AngleAccumulator() {
        --instance_count_;
    }

private:
    double sum_ = 0.0;
    static int instance_count_;
};

// Define static data member at namespace scope.
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
