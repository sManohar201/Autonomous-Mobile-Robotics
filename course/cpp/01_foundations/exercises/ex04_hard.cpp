// Exercise 04 (Hard) — Argument-Dependent Lookup, Overload Resolution Ranking,
//                      Inline Namespaces, and Name Hiding
// ─────────────────────────────────────────────────────────────────────────────

// Q1. ADL: When you call f(x) without namespace qualification, the compiler
//     searches the namespaces associated with each argument's type, in addition
//     to the enclosing scopes. For a type T in namespace N, N is added to the
//     set of associated namespaces searched.

// Q2. `std::cout << my_vec` works without `using namespace my_ns` because ADL
//     sees that my_vec's type belongs to my_ns, so it searches my_ns for
//     operator<<. It finds it there and calls it — no explicit using needed.

// Q3. Overload resolution ranking:
//     Rank 1 (best): exact match / identity conversion
//     Rank 2: lvalue-to-rvalue / qualification (const) conversion
//     Rank 3: promotion (int→long, float→double, bool→int)
//     Rank 4: standard conversion (int→double, double→int)
//     Rank 5 (worst): user-defined conversion

// Q4. `void f(float); void f(double); f(3);`
//     3 is int. int→float is a promotion (rank 3), int→double is a promotion (rank 3).
//     Both are rank 3 → AMBIGUOUS. Adding void f(int) makes rank 1 exact match → f(int) called.

// Q5. inline namespace:
//     Members of an inline namespace are treated as if they were in the enclosing namespace.
//     Without inline: nav_cost::CostMap is ambiguous — it could be v1 or v2.
//     With inline namespace v2: nav_cost::CostMap resolves to v2::CostMap automatically.
//     Use case: ship v2 as the default; existing user code that writes nav_cost::CostMap
//     gets v2 without changes. v1 users can still write nav_cost::v1::CostMap.

// Q6. B::foo(3.14) calls B::foo(int) — name hiding, NOT overloading.
//     When a name is declared in B, it HIDES all overloads of that name from
//     `using namespace A`, even overloads with different signatures.
//     `using namespace A` → names imported but hidden by B::foo declaration.
//     `using A::foo` (declaration, not directive) would ADD A::foo to B's overload set.

// ─────────────────────────────────────────────────────────────────────────────
// PART A — Overload resolution prediction
// ─────────────────────────────────────────────────────────────────────────────

#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>

namespace overload_demo {

void encode(int x)    { std::cout << "encode(int): "    << x           << "\n"; }
void encode(double x) { std::cout << "encode(double): " << std::fixed << std::setprecision(5) << x << "\n"; }

void log_val(float  v) { std::cout << "log_val(float): "  << v << "\n"; }
void log_val(double v) { std::cout << "log_val(double): " << v << "\n"; }

} // namespace overload_demo

void part_a() {
    using namespace overload_demo;

    // PREDICT: encode(int): 42 — exact match to encode(int)
    encode(42);

    // PREDICT: encode(double): 42.00000 — exact match to encode(double)
    encode(42.0);

    // PREDICT: encode(double): 42.00000
    //   float → double is a promotion (rank 3), float → int is a standard conversion (rank 4).
    //   Promotion wins → encode(double) called.
    encode(42.0f);

    // PREDICT: encode(int): 1 — bool → int is an integral promotion (rank 3), exact.
    encode(true);

    // encode(42L) is AMBIGUOUS:
    //   long → int: standard conversion (rank 4)
    //   long → double: standard conversion (rank 4)
    //   Both same rank → AMBIGUOUS. Compiler error if uncommented.
    // encode(42L);

    // PREDICT: log_val(float): 3.14000 — exact match to log_val(float)
    log_val(3.14f);

    // PREDICT: log_val(double): 3.14000 — exact match to log_val(double)
    log_val(3.14);

    // log_val(3) is AMBIGUOUS:
    //   int → float: standard conversion (rank 4)
    //   int → double: standard conversion (rank 4)
    //   Both same rank → AMBIGUOUS.
    // log_val(3);
}

// ─────────────────────────────────────────────────────────────────────────────
// PART B — ADL demonstration with Pose3d
// ─────────────────────────────────────────────────────────────────────────────

namespace geometry {

struct Pose3d {
    double x, y, z;
    double roll, pitch, yaw;
};

// operator<< in geometry:: — ADL finds it when printing a Pose3d.
std::ostream& operator<<(std::ostream& os, const Pose3d& p) {
    os << "x=" << p.x << " y=" << p.y << " z=" << p.z
       << " roll=" << p.roll << " pitch=" << p.pitch << " yaw=" << p.yaw;
    return os;
}

// operator== in geometry:: — ADL finds it for (p1 == p2).
bool operator==(const Pose3d& a, const Pose3d& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z &&
           a.roll == b.roll && a.pitch == b.pitch && a.yaw == b.yaw;
}

} // namespace geometry

// If operator<< were in the GLOBAL namespace, ADL would NOT find it for
// `std::cout << pose`. ADL only searches associated namespaces of the arguments'
// types. Pose3d is in geometry::, so only geometry:: (and its enclosing
// namespaces, up to global) are searched. A global-namespace operator<< would
// be found only via regular unqualified lookup, not ADL. In practice it would
// be found, but relying on global-namespace operators is fragile.

// ─────────────────────────────────────────────────────────────────────────────
// PART C — Inline namespace for API versioning
// ─────────────────────────────────────────────────────────────────────────────

namespace nav_cost {

    namespace v1 {
        struct CostMap {
            int width, height;
            double resolution;
        };

        double compute_cost(const CostMap& m, int x, int y);
    }

    inline namespace v2 {
        struct CostMap {
            int    width, height;
            double resolution;
            double* data;
        };

        double compute_cost(const CostMap& m, int x, int y);
        double compute_cost(const CostMap& m, int x, int y, double clearance_m);
    }

} // namespace nav_cost

// When v3 ships as the new inline namespace, existing user code calling
// nav_cost::CostMap and nav_cost::compute_cost gets v3 automatically.
// Users who need v2 explicitly write nav_cost::v2::CostMap.
// This allows ABI-compatible evolution without forcing users to update call sites.

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
    using namespace utils;

    // robot::configure(int) HIDES all utils::configure overloads — not just (int).
    // 3.14 (double) does not find utils::configure(double); it converts to int.
    void configure(int timeout_ms) {
        std::cout << "robot::configure(int): " << timeout_ms << "\n";
    }
}

// robot::configure(3.14) is NAME HIDING: robot::configure(int) hides ALL
// utils:: overloads that were brought in by `using namespace utils`.
// FIX: add `using utils::configure;` inside robot:: to inject the overload set
// alongside robot::configure(int), converting hiding → overloading.

// ─────────────────────────────────────────────────────────────────────────────
// nav_cost implementations
// ─────────────────────────────────────────────────────────────────────────────

double nav_cost::v1::compute_cost(const nav_cost::v1::CostMap& m, int x, int y) {
    return (x + y) * m.resolution;
}

double nav_cost::v2::compute_cost(const nav_cost::v2::CostMap& m, int x, int y) {
    if (m.data) return m.data[y * m.width + x];
    return (x + y) * m.resolution;
}

double nav_cost::v2::compute_cost(const nav_cost::v2::CostMap& m, int x, int y, double clearance_m) {
    return nav_cost::v2::compute_cost(m, x, y) + clearance_m;
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

    // Part A
    part_a();
    std::cout << "\n";

    // Part B — ADL
    geometry::Pose3d p1{1.0, 2.0, 0.0, 0.0, 0.0, 3.14159265358979 / 4.0};
    geometry::Pose3d p2{1.0, 2.0, 0.0, 0.0, 0.0, 0.0};

    std::cout << "Pose3d via ADL: " << p1 << "\n";
    std::cout << "p1 == p2: " << (p1 == p2) << "\n";
    std::cout << "\n";

    // Part C — inline namespace
    nav_cost::CostMap m2{};
    m2.width  = 10;
    m2.height = 10;
    m2.resolution = 0.1;
    m2.data = nullptr;

    std::cout << "nav_cost version: CostMap resolves to v2\n";
    std::cout << "v2 cost at (0,0) = " << nav_cost::compute_cost(m2, 0, 0) << "\n";
    std::cout << "v2 cost at (1,2) with clearance 1.5 = "
              << nav_cost::compute_cost(m2, 1, 2, 1.5) << "\n";

    nav_cost::v1::CostMap old_m{10, 10, 0.05};
    std::cout << "v1 CostMap width: " << old_m.width << "\n";
    std::cout << "\n";

    // Part D — name hiding
    std::cout << "robot::configure(int) called — utils::configure(double) was hidden:\n";
    robot::configure(3);

    return 0;
}
