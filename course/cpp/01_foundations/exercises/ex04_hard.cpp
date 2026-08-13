// Exercise 04 (Hard) — Argument-Dependent Lookup, Overload Resolution Ranking,
//                      Inline Namespaces, and Name Hiding
// ─────────────────────────────────────────────────────────────────────────────
//
// CONTEXT:
//   Eigen — the foundational linear algebra library used by every robotics
//   stack (ROS2, MoveIt, nav2) — relies entirely on ADL for its operators.
//   Engineers who don't understand ADL struggle to write geometry types that
//   interoperate cleanly with std::cout and standard algorithms. Overload
//   resolution traps are a leading cause of "wrong function called" bugs in
//   production robotics code.
//
// ─────────────────────────────────────────────────────────────────────────────
// PRE-CODING RESEARCH QUESTIONS — Answer in the spaces provided.
// ─────────────────────────────────────────────────────────────────────────────

// Q1. What is Argument-Dependent Lookup (ADL)?
//     When you call `f(x)` without namespace qualification, which namespaces
//     does the compiler search in addition to the current scope?
//
//     YOUR ANSWER:
//     TODO

// Q2. Why does `std::cout << my_vec` work WITHOUT `using namespace my_ns;`
//     if `operator<<` is defined in the same namespace as `my_vec`'s type?
//
//     YOUR ANSWER:
//     TODO

// Q3. Overload resolution ranking (best-to-worst). Fill in the blanks:
//     Rank 1 (best): exact match
//     Rank 2: ___________
//     Rank 3: ___________
//     Rank 4: ___________
//     Rank 5 (worst): user-defined conversion
//
//     YOUR ANSWER:
//     TODO

// Q4. `void f(float); void f(double); f(3);`
//     Which overload is called? Why?
//     What changes if you add `void f(int);`?
//
//     YOUR ANSWER:
//     TODO

// Q5. What does `inline namespace` do that a regular nested namespace does not?
//     Give a real-world use case (API versioning in a robotics library).
//
//     YOUR ANSWER:
//     TODO

// Q6. Name hiding:
//     namespace A { void foo(int); void foo(double); }
//     namespace B { using namespace A; void foo(int); }
//     What does `B::foo(3.14)` call? Is this overloading or hiding?
//     Explain the difference between `using namespace A` and `using A::foo`.
//
//     YOUR ANSWER:
//     TODO

// ─────────────────────────────────────────────────────────────────────────────
// PART A — Overload resolution prediction
//
// For each call below, predict which overload is called (or write AMBIGUOUS/
// ERROR). Write your reasoning as a comment on the PREDICT line, then
// uncomment the call. Leave ambiguous calls commented out with explanation.
// ─────────────────────────────────────────────────────────────────────────────

#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>

namespace overload_demo {

void encode(int x)    { std::cout << "encode(int): "    << x           << "\n"; } // E1
void encode(double x) { std::cout << "encode(double): " << std::fixed << std::setprecision(5) << x << "\n"; } // E2

void log_val(float  v) { std::cout << "log_val(float): "  << v << "\n"; } // L1
void log_val(double v) { std::cout << "log_val(double): " << v << "\n"; } // L2

} // namespace overload_demo

void part_a() {
    using namespace overload_demo;

    // PREDICT: TODO — encode(42) calls which overload and why?
    encode(42);

    // PREDICT: TODO — encode(42.0) calls which?
    encode(42.0);

    // PREDICT: TODO — encode(42.0f)?
    //   (float → double is a promotion, outranking standard conversion to int)
    encode(42.0f);

    // PREDICT: TODO — encode(true)?
    //   (bool → int is an integral promotion)
    encode(true);

    // encode(42L) — long argument, two overloads:
    //   E1: long → int  is an integral conversion (rank 3)
    //   E2: long → double is a floating-integral conversion (rank 3)
    //   Both are the same rank → AMBIGUOUS.
    // PREDICT: AMBIGUOUS — leave commented out, explain below:
    //
    // TODO: YOUR EXPLANATION of why encode(42L) is ambiguous
    // encode(42L);  // AMBIGUOUS: long→int and long→double are both standard conversions

    // PREDICT: TODO — log_val(3.14f)?
    log_val(3.14f);

    // PREDICT: TODO — log_val(3.14)?
    log_val(3.14);

    // log_val(3) — int argument:
    //   L1: int → float  is a standard conversion (rank 3)
    //   L2: int → double is a standard conversion (rank 3)
    //   AMBIGUOUS.
    // PREDICT: AMBIGUOUS — leave commented out:
    //
    // TODO: YOUR EXPLANATION
    // log_val(3);  // AMBIGUOUS
}

// ─────────────────────────────────────────────────────────────────────────────
// PART B — ADL demonstration with Pose3d
//
// Implement operator<< and operator== in the geometry namespace.
// Show in main that they work via ADL without any `using` declaration.
// ─────────────────────────────────────────────────────────────────────────────

namespace geometry {

struct Pose3d {
    double x, y, z;
    double roll, pitch, yaw;
};

// TODO: implement operator<< in the geometry namespace so ADL finds it.
// Remove this stub and replace with your implementation.
std::ostream& operator<<(std::ostream& os, const Pose3d& p) {
    (void)p;
    os << "[stub — implement me]";
    return os;
}

// TODO: implement operator== in the geometry namespace.
// Remove this stub and replace with your implementation.
bool operator==(const Pose3d& a, const Pose3d& b) {
    (void)a; (void)b;
    return false; // stub
}

} // namespace geometry

// THINK: If you defined operator<< in the GLOBAL namespace instead of
//        geometry::, would ADL still find it for `std::cout << pose`?
//        Why or why not? Answer in a comment below:
//
// TODO: YOUR ANSWER

// ─────────────────────────────────────────────────────────────────────────────
// PART C — Inline namespace for API versioning
//
// Build a versioned nav_cost library.
// Show that nav_cost::CostMap resolves to v2 and nav_cost::v1::CostMap
// still works.
// ─────────────────────────────────────────────────────────────────────────────

namespace nav_cost {

    namespace v1 {
        struct CostMap {
            int width, height;
            double resolution;
        };

        // TODO: implement — simple formula: return (x + y) * resolution
        double compute_cost(const CostMap& m, int x, int y);
    }

    // This namespace should be `inline namespace v2` — the `inline` keyword is
    // the key task for Part C. It is already set here; understand WHY it must
    // be inline and what changes if you remove the `inline` keyword.
    inline namespace v2 {
        struct CostMap {
            int    width, height;
            double resolution;
            double* data;  // pointer to flattened cost grid (may be nullptr)
        };

        // TODO: implement — if data is not null: return data[y * m.width + x]
        //                   else: return (x + y) * resolution
        double compute_cost(const CostMap& m, int x, int y);

        // TODO: implement — add clearance_m penalty: cost + clearance_m
        double compute_cost(const CostMap& m, int x, int y, double clearance_m);
    }

} // namespace nav_cost

// THINK: What happens to existing user code (that calls nav_cost::compute_cost)
//        when the library ships v3 as the new inline namespace?
//        Answer in a comment below:
//
// TODO: YOUR ANSWER

// ─────────────────────────────────────────────────────────────────────────────
// PART D — Name hiding trap
//
// Demonstrate that robot::configure(3.14) does NOT call utils::configure(double)
// even though utils::configure is brought in via `using namespace utils`.
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
    using namespace utils;  // brings in all three utils::configure overloads

    // Declaring configure(int) in robot:: HIDES all utils::configure overloads,
    // not just the (int) one. This is NAME HIDING, not overloading.
    void configure(int timeout_ms) {
        std::cout << "robot::configure(int): " << timeout_ms << "\n";
    }
}

// THINK: Is robot::configure(3.14) overloading or hiding? Explain.
//        How would you fix this so robot::configure(double) falls through
//        to utils::configure(double)?
//        Answer in a comment below:
//
// TODO: YOUR ANSWER

// ─────────────────────────────────────────────────────────────────────────────
// nav_cost implementations (add bodies here or above in the namespace)
// ─────────────────────────────────────────────────────────────────────────────

// TODO: implement nav_cost::v1::compute_cost
double nav_cost::v1::compute_cost(const nav_cost::v1::CostMap& m, int x, int y) {
    (void)m; (void)x; (void)y;
    return 0.0; // stub
}

// TODO: implement nav_cost::v2::compute_cost (two overloads)
double nav_cost::v2::compute_cost(const nav_cost::v2::CostMap& m, int x, int y) {
    (void)m; (void)x; (void)y;
    return 0.0; // stub
}
double nav_cost::v2::compute_cost(const nav_cost::v2::CostMap& m, int x, int y, double clearance_m) {
    (void)m; (void)x; (void)y; (void)clearance_m;
    return 0.0; // stub
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

    // These must work via ADL — no `using namespace geometry` allowed here
    std::cout << "Pose3d via ADL: " << p1 << "\n";
    std::cout << "p1 == p2: " << (p1 == p2) << "\n";
    std::cout << "\n";

    // Part C — inline namespace
    // nav_cost::CostMap should resolve to v2 (because v2 is inline)
    nav_cost::CostMap m2{};  // TODO: make v2 the inline namespace so this works
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
    robot::configure(3);   // calls robot::configure(int) — fine
    // robot::configure(3.14); // This would call robot::configure(int) NOT utils::configure(double)
    //                         // because robot::configure(int) HIDES all utils:: overloads.
    //                         // 3.14 (double) converts to int → robot::configure(int).
    //                         // Uncomment to observe.

    return 0;
}
