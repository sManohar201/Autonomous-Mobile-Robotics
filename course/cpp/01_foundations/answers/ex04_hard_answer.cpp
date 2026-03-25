// ANSWER — Exercise 04 (Hard): Argument-Dependent Lookup, Overload Resolution
//                              Ranking, Inline Namespaces, and Name Hiding
// ─────────────────────────────────────────────────────────────────────────────

// ── Q1 ────────────────────────────────────────────────────────────────────────
// Argument-Dependent Lookup (ADL), also called Koenig lookup:
//
// When you call an unqualified function `f(x)`, the compiler searches:
//   1. The current scope and all enclosing scopes (normal name lookup).
//   2. All namespaces associated with the TYPES of the arguments.
//
// "Associated namespaces" for a type T are:
//   - The namespace where T is defined.
//   - If T is a class: all base class namespaces, and the enclosing namespaces
//     of the class template (if T is a template specialization).
//   - If T is a function pointer: the parameter/return type namespaces.
//
// ADL is applied IN ADDITION to ordinary lookup. All candidates from both
// lookups form the overload set.

// ── Q2 ────────────────────────────────────────────────────────────────────────
// `std::cout << my_vec` works without `using namespace my_ns` because ADL
// kicks in for the argument `my_vec`. Its type is `my_ns::Vec`. The compiler
// adds `my_ns` to the set of namespaces to search, and finds
// `my_ns::operator<<(std::ostream&, const my_ns::Vec&)` there.
//
// The expression `std::cout << my_vec` desugars to the function call
// `operator<<(std::cout, my_vec)`. The second argument's type `my_ns::Vec`
// causes `my_ns` to be searched → the operator is found.
// Without ADL, Eigen's matrix output and every robotics geometry type would
// require a `using` declaration before every print statement. ADL makes this
// just work.

// ── Q3 ────────────────────────────────────────────────────────────────────────
// Overload resolution ranking (best-to-worst):
//   Rank 1 (best): Exact match — no conversion needed at all
//   Rank 2: Promotion — bool→int, float→double, small integral→int
//   Rank 3: Standard conversion — int→double, int→long, pointer conversions
//   Rank 4: Qualification adjustment — T* → const T* (adding const)
//   Rank 5 (worst): User-defined conversion — via converting constructor or
//                   conversion operator
//
// If two overloads tie at the same rank for all arguments, the call is AMBIGUOUS
// and does not compile.

// ── Q4 ────────────────────────────────────────────────────────────────────────
// `void f(float); void f(double); f(3);`
// Integer literal 3 has type int.
//   - f(float):  int → float  is a STANDARD CONVERSION (rank 3)
//   - f(double): int → double is a STANDARD CONVERSION (rank 3)
//   Both are the same rank → AMBIGUOUS. Compilation error.
//
// Adding `void f(int)`: now there is an EXACT MATCH (rank 1). f(int) is
// called. The ambiguity disappears because exact match outranks everything.

// ── Q5 ────────────────────────────────────────────────────────────────────────
// `inline namespace` makes names in the nested namespace directly accessible
// in the enclosing namespace, AS IF they were declared there. Specifically:
//   - `ns::inner::foo` is also directly accessible as `ns::foo`.
//   - ADL finds the inline namespace's names.
//   - Template specializations in the outer namespace still work.
//
// Regular nested namespace: you must qualify with `ns::inner::foo`.
//
// Real-world use case — API versioning:
//   namespace geometry {
//       namespace v1 { struct Vec2 { float x, y; }; }
//       inline namespace v2 { struct Vec2 { double x, y; }; }
//   }
//   geometry::Vec2 v;  // resolves to v2::Vec2 — the current version
//   geometry::v1::Vec2 old;  // still accessible for compatibility
// When v3 ships, change `inline namespace v2` → `namespace v2` and add
// `inline namespace v3`. All existing `geometry::Vec2` users automatically
// get v3. Old v2 code can pin to `geometry::v2::Vec2`.

// ── Q6 ────────────────────────────────────────────────────────────────────────
// Given:
//   namespace A { void foo(int); void foo(double); }
//   namespace B { using namespace A; void foo(int); }
//
// `B::foo(3.14)` calls: B::foo(int) — NOT A::foo(double).
//
// This is NAME HIDING, not overloading:
//   `using namespace A` does NOT add A's names to B's overload set. It only
//   makes A's names visible in B through unqualified lookup IF B does not
//   already declare a name `foo`. But B declares `foo(int)`, which HIDES ALL
//   overloads of `foo` from namespace A — regardless of parameter types.
//   So `B::foo(3.14)` finds only `B::foo(int)`, converts 3.14→int, and calls it.
//
// `using namespace A` vs `using A::foo`:
//   - `using namespace A`: A's names are found by unqualified lookup AFTER
//     B's own names. B's declarations HIDE them (name hiding).
//   - `using A::foo`:  introduces A::foo(int) and A::foo(double) as
//     individual overload candidates INTO B's scope. Now `B::foo(3.14)` is
//     an overload resolution over {B::foo(int), A::foo(int), A::foo(double)}
//     and A::foo(double) would win as exact match.
//
// Fix: replace `using namespace A` with `using A::foo;` in namespace B.

// ─────────────────────────────────────────────────────────────────────────────

#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// PART A — Overload resolution
// ─────────────────────────────────────────────────────────────────────────────

namespace overload_demo {

void encode(int x) {
    std::cout << "encode(int): " << x << "\n";
}
void encode(double x) {
    std::cout << "encode(double): " << std::fixed << std::setprecision(5) << x << "\n";
}

void log_val(float  v) { std::cout << "log_val(float): "  << v << "\n"; }
void log_val(double v) { std::cout << "log_val(double): " << v << "\n"; }

} // namespace overload_demo

void part_a() {
    using namespace overload_demo;

    // encode(42): literal 42 is type int → EXACT MATCH for encode(int) → E1
    encode(42);

    // encode(42.0): literal 42.0 is type double → EXACT MATCH for encode(double) → E2
    encode(42.0);

    // encode(42.0f): literal 42.0f is type float.
    //   - float → int:    standard conversion (rank 3)
    //   - float → double: PROMOTION (rank 2)
    // Rank 2 beats rank 3 → encode(double) is called via promotion.
    encode(42.0f);

    // encode(true): literal true is type bool.
    //   - bool → int:    PROMOTION (rank 2)
    //   - bool → double: promotion to int then standard conversion to double
    //                    (promotion is rank 2, so bool→int wins)
    // encode(int) called via bool→int integral promotion.
    encode(true);

    // encode(42L): 42L has type long.
    //   - long → int:    integral conversion (rank 3, standard conversion)
    //   - long → double: floating-integral conversion (rank 3, standard conversion)
    //   Both are rank 3 → AMBIGUOUS. Does not compile. Left commented out.
    // encode(42L);  // AMBIGUOUS: long→int and long→double both rank 3

    // log_val(3.14f): 3.14f is type float → EXACT MATCH for log_val(float) → L1
    log_val(3.14f);

    // log_val(3.14): 3.14 is type double → EXACT MATCH for log_val(double) → L2
    log_val(3.14);

    // log_val(3): 3 is type int.
    //   - int → float:  standard conversion (rank 3)
    //   - int → double: standard conversion (rank 3)
    //   AMBIGUOUS — does not compile. Left commented out.
    // log_val(3);  // AMBIGUOUS: int→float and int→double both rank 3
}

// ─────────────────────────────────────────────────────────────────────────────
// PART B — ADL demonstration
// ─────────────────────────────────────────────────────────────────────────────

namespace geometry {

struct Pose3d {
    double x, y, z;
    double roll, pitch, yaw;
};

// operator<< defined IN geometry namespace.
// ADL: when the compiler sees `os << pose` (i.e., operator<<(os, pose)),
// the second argument is geometry::Pose3d → compiler searches geometry:: for
// operator<< → finds this function. No `using namespace geometry` needed.
std::ostream& operator<<(std::ostream& os, const Pose3d& p) {
    // Save and restore stream state so we don't clobber caller's precision.
    std::ios_base::fmtflags saved_flags = os.flags();
    std::streamsize         saved_prec  = os.precision();
    os << std::fixed << std::setprecision(3);
    os << "x=" << p.x
       << " y=" << p.y
       << " z=" << p.z
       << " roll=" << p.roll
       << " pitch=" << p.pitch
       << " yaw=" << p.yaw;
    os.flags(saved_flags);
    os.precision(saved_prec);
    return os;
}

// operator== defined IN geometry namespace.
// ADL: `p1 == p2` calls operator==(p1, p2). Both args are geometry::Pose3d
// → geometry:: is searched → this function found.
bool operator==(const Pose3d& a, const Pose3d& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z
        && a.roll == b.roll && a.pitch == b.pitch && a.yaw == b.yaw;
}

} // namespace geometry

// THINK: If operator<< were in the GLOBAL namespace instead of geometry::,
// would ADL still work?
//
// YES, but only because the global namespace is always searched by ordinary
// (non-ADL) lookup. ADL adds EXTRA namespaces; it does not remove the global
// namespace from consideration. However, placing operator<< in the global
// namespace is bad practice: it pollutes the global namespace and can cause
// unintended overload ambiguities with other global operator<< overloads.
// The correct design is always: operator<< in the same namespace as the type.

// ─────────────────────────────────────────────────────────────────────────────
// PART C — Inline namespace for API versioning
// ─────────────────────────────────────────────────────────────────────────────

namespace nav_cost {

    namespace v1 {
        struct CostMap { int width, height; double resolution; };
        double compute_cost(const CostMap& m, int x, int y) {
            return (x + y) * m.resolution;
        }
    }

    // `inline namespace v2`: all v2 names are accessible as nav_cost:: names.
    // nav_cost::CostMap   → v2::CostMap
    // nav_cost::compute_cost → v2::compute_cost (both overloads)
    //
    // When a future v3 is ready, the library author changes this to
    // `namespace v2` and adds `inline namespace v3`. All existing user code
    // that uses `nav_cost::CostMap` automatically gets v3. Existing code
    // pinned to `nav_cost::v2::CostMap` still compiles unchanged.
    inline namespace v2 {
        struct CostMap { int width, height; double resolution; double* data; };

        double compute_cost(const CostMap& m, int x, int y) {
            if (m.data) return m.data[y * m.width + x];
            return (x + y) * m.resolution;
        }

        double compute_cost(const CostMap& m, int x, int y, double clearance_m) {
            return compute_cost(m, x, y) + clearance_m;
        }
    }

} // namespace nav_cost

// ─────────────────────────────────────────────────────────────────────────────
// PART D — Name hiding trap
// ─────────────────────────────────────────────────────────────────────────────

namespace utils {
    void configure(int timeout_ms) {
        std::cout << "utils::configure(int): " << timeout_ms << "\n";
    }
    void configure(double frequency_hz) {
        std::cout << "utils::configure(double): " << frequency_hz << "\n";
    }
    void configure(const std::string& name) {
        std::cout << "utils::configure(string): " << name << "\n";
    }
}

namespace robot {
    // `using namespace utils` brings utils' names into robot:: via unqualified
    // lookup, but they appear LOWER in the lookup hierarchy than robot's own
    // declarations. When robot:: declares ANY overload of `configure`, that
    // single declaration HIDES ALL overloads from utils:: — not just the one
    // with matching parameters. This is NAME HIDING, not overloading.
    //
    // Effect: robot::configure(double) does NOT exist. Calling
    // robot::configure(3.14) finds only robot::configure(int) and converts
    // 3.14 → int.
    //
    // FIX: Replace `using namespace utils` with `using utils::configure;`
    // This introduces all three utils::configure overloads as candidates in
    // robot::, forming a genuine overload set with robot::configure(int).
    // Then robot::configure(3.14) would call utils::configure(double) via
    // exact match.
    using namespace utils;

    void configure(int timeout_ms) {
        std::cout << "robot::configure(int): " << timeout_ms << "\n";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// EXPECTED OUTPUT:
//   encode(int): 42
//   encode(double): 42.00000
//   encode(double): 42.00000
//   encode(int): 1
//   log_val(float): 3.14000
//   log_val(double): 3.14000
//
//   Pose3d via ADL: x=1.000 y=2.000 z=0.000 roll=0.000 pitch=0.000 yaw=0.785
//   p1 == p2: 0
//
//   nav_cost version: CostMap resolves to v2
//   v2 cost at (0,0) = 0.00000
//   v2 cost at (1,2) with clearance 1.5 = 1.80000
//   v1 CostMap width: 10
//
//   robot::configure(int) called — utils::configure(double) was hidden:
//   robot::configure(int): 3
// ─────────────────────────────────────────────────────────────────────────────

int main() {
    std::cout << std::fixed << std::setprecision(5);

    part_a();
    std::cout << "\n";

    // Part B — ADL: no `using namespace geometry` here
    geometry::Pose3d p1{1.0, 2.0, 0.0, 0.0, 0.0, 3.14159265358979 / 4.0};
    geometry::Pose3d p2{1.0, 2.0, 0.0, 0.0, 0.0, 0.0};
    std::cout << "Pose3d via ADL: " << p1 << "\n";
    std::cout << "p1 == p2: " << (p1 == p2) << "\n";
    std::cout << "\n";

    // Part C — inline namespace
    nav_cost::CostMap m2{};   // resolves to nav_cost::v2::CostMap (inline)
    m2.width  = 10;
    m2.height = 10;
    m2.resolution = 0.1;
    m2.data = nullptr;

    std::cout << "nav_cost version: CostMap resolves to v2\n";
    // v2 cost at (0,0): data=nullptr, (0+0)*0.1 = 0.0
    // v2 cost at (1,2) + clearance 1.5: (1+2)*0.1 + 1.5 = 0.3 + 1.5 = 1.8
    std::cout << "v2 cost at (0,0) = "
              << nav_cost::compute_cost(m2, 0, 0) << "\n";
    std::cout << "v2 cost at (1,2) with clearance 1.5 = "
              << nav_cost::compute_cost(m2, 1, 2, 1.5) << "\n";

    nav_cost::v1::CostMap old_m{10, 10, 0.05};
    std::cout << "v1 CostMap width: " << old_m.width << "\n";
    std::cout << "\n";

    // Part D — name hiding
    std::cout << "robot::configure(int) called — utils::configure(double) was hidden:\n";
    // robot::configure(3) → robot::configure(int): unambiguous
    robot::configure(3);
    // robot::configure(3.14) would call robot::configure(int) because
    // utils::configure(double) is HIDDEN. 3.14 converts to int = 3.
    // Uncomment to observe:
    // robot::configure(3.14);

    return 0;
}
