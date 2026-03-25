// ANSWER — Exercise 01 (Hard): One Definition Rule, Linkage, Inline, and
//                              Symbol Visibility
// ─────────────────────────────────────────────────────────────────────────────

// ── Q1 ────────────────────────────────────────────────────────────────────────
// The ODR says: at most one definition per TU, AND exactly one definition in
// the entire program for entities with external linkage — UNLESS they fall
// into a special ODR-exempt category.
//
// May have MORE THAN ONE definition across TUs (provided token-identical):
//   (b) inline function          — ODR-exempt; each TU needs a copy to inline
//   (c) class definition         — ODR-exempt; headers include them everywhere
//   (d) constexpr function       — implicitly inline; ODR-exempt
//   (e) constexpr variable (C++17) — inline variable; ODR-exempt
//
// Must have EXACTLY ONE definition in the whole program:
//   (a) non-inline function      — one definition rule applies strictly
//   (f) non-const global variable — external linkage, exactly one definition
//
// Root reason: inline/constexpr entities are "ODR-exempt" — the compiler is
// permitted (required, in practice) to emit a copy in every TU that needs one,
// as long as every definition is token-for-token identical. The linker folds
// them (COMDAT/weak symbols). The non-inline, non-constexpr entities are
// "strong symbols" and the linker rejects duplicates.

// ── Q2 ────────────────────────────────────────────────────────────────────────
// (a) Compiler processes sensor.cpp: sees wrap_angle() definition, emits a
//     STRONG symbol 'T _Z10wrap_angled' in sensor.o.
// (b) Compiler processes main.cpp:   same — emits the same strong symbol in
//     main.o. No error yet; each TU is valid on its own.
// (c) Linker receives sensor.o + main.o and sees TWO strong symbols with the
//     same mangled name. Strong + Strong = duplicate definition error.
// (d) Error (at LINK time, not compile time):
//       ld: error: duplicate symbol '_Z10wrap_angled'
//           first defined in sensor.o
//           also defined in main.o
//     The compile step succeeds for each TU individually; only the linker
//     catches the violation.
// Fix: add the `inline` keyword (or move the body to a .cpp file).

// ── Q3 ────────────────────────────────────────────────────────────────────────
// (a) EXTERNAL LINKAGE: the entity is visible to ALL translation units.
//     Examples:
//       - Non-const global variable: `int counter = 0;` at namespace scope
//       - Non-inline function at namespace scope: `double compute(double x);`
//
// (b) INTERNAL LINKAGE: the entity is visible only within THIS translation unit.
//     Two ways to achieve it:
//       1. `static` keyword at file scope: `static double helper(double x) {...}`
//       2. Anonymous namespace:            `namespace { double helper(double x){...} }`
//
// (c) NO LINKAGE: the entity cannot be referred to from any other scope at all.
//     Example: a local variable inside a function body.
//       void foo() { int x = 5; }  // x has no linkage

// ── Q4 ────────────────────────────────────────────────────────────────────────
// nm symbol type codes (lowercase = local/internal, uppercase = global/external):
//   T (or t) : Text section — defined code (strong symbol)
//   U         : Undefined — referenced but not yet defined (supplied at link time)
//   W (or w) : Weak symbol — defined but can be overridden by a strong symbol
//              (inline functions and template instantiations typically appear as W)
//
// What nm shows for clamp_range in an anonymous namespace:
//   clamp_range will appear as a LOWERCASE 't' symbol — local text — because
//   anonymous-namespace entities have internal linkage. The linker will not
//   attempt to resolve or export it.  The mangled name will contain the
//   unnamed-namespace marker (e.g., `(anonymous namespace)::clamp_range(double, double)`
//   when demangled with nm -C).  It will NOT be visible to other object files.
//
// For wrap_to_pi (inline): appears as W or w — a weak symbol. Multiple TUs
//   may emit a definition; the linker picks one and discards the rest.
// For deg_to_rad (constexpr, implicitly inline): same — W.

// ── Q5 ────────────────────────────────────────────────────────────────────────
// (a) C++ mangles function names to encode their parameter types and namespace
//     into the symbol name. This is necessary because C++ supports overloading:
//     `void send(int)` and `void send(double)` are two different functions and
//     the linker must distinguish them. C has no overloading, so no mangling.
//
// (b) `extern "C"` disables name mangling for the enclosed declaration.
//     The symbol name in the object file becomes the plain C identifier.
//     Needed for C++ → C interop because:
//       - A C driver #includes a plain C header (no C++ awareness).
//       - The C driver calls the function by its unmangled C name.
//       - If the C++ library emits a mangled name, the linker cannot match it.
//     With extern "C" { void init_sensor(); } the C++ compiler emits the
//     symbol as `init_sensor` (not `_Z11init_sensorv`), which the C linker finds.

// ── Q6 ────────────────────────────────────────────────────────────────────────
// (a) `static` at file scope gives the variable INTERNAL LINKAGE — it is
//     only visible within the translation unit where it is defined.
//
// (b) THREE copies — one per TU that includes the header. Each TU gets its
//     own private dropped_packets variable that is completely independent.
//
// (c) This is a bug when you intend a shared counter: incrementing
//     dropped_packets in node.cpp does NOT affect the copy in logger.cpp.
//     No error is reported; the program compiles and runs, but each TU
//     silently tracks its own count. Debugging this is extremely painful.
//
//     CORRECT pattern for a shared counter:
//       // config.hpp
//       extern int dropped_packets;   // declaration only — tells compiler it exists
//       // config.cpp
//       int dropped_packets = 0;      // ONE definition — all TUs share this

// ── PART A — Bug Hunt ─────────────────────────────────────────────────────────

// A1 — utils.hpp: wrap_angle defined with a body, included from 2+ TUs.
//   BUG: non-inline function with external linkage defined in a header.
//   Rule violated: ODR — exactly one definition per program for non-inline functions.
//   Error: "multiple definition of `wrap_angle(double)`" — LINKER error.
//   Fix: add `inline` keyword to wrap_angle, OR move body to utils.cpp.
//   Note: MAX_RANGE is `constexpr` — since C++17 constexpr variables are
//         implicitly inline, so that line is fine.

// A2 — config.hpp: `static int dropped_packets = 0;`
//   NOT a linker error (static = internal linkage, each TU gets its own copy).
//   BUT a LOGIC BUG: 3 TUs → 3 independent counters that never share state.
//   No compiler/linker error; broken program behavior.
//   Fix: `extern int dropped_packets;` in header + `int dropped_packets = 0;` in one .cpp.

// A3 — shared.hpp: `int total_frames = 0;`
//   BUG: non-const global variable with external linkage defined in a header.
//   Two TUs include it → two strong T symbols for `total_frames`.
//   Error: "multiple definition of `total_frames`" — LINKER error.
//   Fix: `extern int total_frames;` in header + `int total_frames = 0;` in one .cpp.

// A4 — math_utils.hpp: `namespace { double sq(double x) { return x * x; } }`
//   NOT a bug in terms of linking. Anonymous namespace gives internal linkage
//   to sq. Each of the 5 TUs gets its own private copy — no ODR violation.
//   The linker never sees conflicting strong symbols.
//   Potential CONCERN: 5 copies of sq in the binary (code bloat), but this is
//   intentional for file-private helpers.
//   No error at any build stage.

// ─────────────────────────────────────────────────────────────────────────────

#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>

constexpr double PI = 3.14159265358979323846;

// ── B1: constexpr deg_to_rad ──────────────────────────────────────────────────
// This lives in a header (or here, in one TU) without violating ODR because
// constexpr functions are IMPLICITLY INLINE. The compiler is allowed to emit
// one copy per TU, and the linker folds them. If called in a constant
// expression, the compiler evaluates it at compile time and emits no code at all.
constexpr double deg_to_rad(double degrees) {
    return degrees * (PI / 180.0);
}

// ── B2: inline wrap_to_pi ─────────────────────────────────────────────────────
// Must be inline because it has a full definition (body). If this lived in a
// header included by many TUs, the `inline` keyword tells the compiler that
// multiple definitions are intentional and ODR-exempt. Without it, each TU
// that includes the header produces a strong symbol → linker error.
inline double wrap_to_pi(double rad) {
    double r = rad + PI;
    r = std::fmod(r, 2.0 * PI);
    if (r < 0.0) r += 2.0 * PI;
    return r - PI;
}

// ── B3: clamp_range with internal linkage ────────────────────────────────────
// Anonymous namespace vs. `static` at file scope:
//   - Both give the name internal linkage — not visible outside this TU.
//   - KEY DIFFERENCE: anonymous namespace can contain types (classes, enums,
//     type aliases) and gives them internal linkage too, which `static` cannot
//     do for types. For functions they are equivalent in practice.
//   - Style: anonymous namespace is preferred in modern C++ because it works
//     for all kinds of declarations, not just objects and functions.
namespace {
    double clamp_range(double r, double max_r) {
        if (r < 0.0)    return 0.0;
        if (r > max_r)  return max_r;
        return r;
    }
}

// ── B4: AngleAccumulator ──────────────────────────────────────────────────────
// static data members:
//   - Declared inside the class body (declaration only).
//   - Must have EXACTLY ONE definition at namespace scope outside the class.
//     This is mandated by the ODR for non-inline static data members.
//   - In C++17 you can avoid the out-of-class definition by making the member
//     `inline static int instance_count_ = 0;` — inline variables are ODR-exempt.
//   - Here we use the classic pre-C++17 pattern (definition below the class).
class AngleAccumulator {
public:
    explicit AngleAccumulator(double initial = 0.0)
        : sum_(wrap_to_pi(initial))
    {
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
    double sum_;
    static int instance_count_;  // declaration — storage defined below
};

// ONE definition of the static data member, at namespace scope.
// ODR clause: non-inline static data members require exactly one definition
// in the entire program. If this were in a header, including it in two .cpp
// files would be a linker error — same as A3 above.
int AngleAccumulator::instance_count_ = 0;

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
