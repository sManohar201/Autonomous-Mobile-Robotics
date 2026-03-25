// Exercise 01 — Invariant-enforcing class design: Covariance2d
//
// Context
// -------
// In an Extended Kalman Filter (EKF) the state uncertainty is tracked as a
// covariance matrix.  For a 2D position estimate the covariance is a 2×2
// symmetric positive-definite (SPD) matrix:
//
//   P = [ var_x   cov_xy ]
//       [ cov_xy  var_y  ]
//
// "Symmetric"         means P[0][1] == P[1][0]  (the class stores cov_xy once).
// "Positive-definite" means every principal minor is positive:
//     1st minor: var_x > 0
//     2nd minor (determinant): var_x*var_y - cov_xy^2 > 0
//
// If P is not SPD the filter is mathematically invalid — it can predict negative
// variance, which has no physical meaning.  The constructor must GUARANTEE the
// invariant; if it would be violated, throw std::invalid_argument.
//
// Before you write any code, answer these questions in comments:
//   Q1. Why does correlation = +1 or -1 break the positive-definite property?
//   Q2. If var_x = 4, var_y = 9, cov_xy = 6, is the matrix PD?  Verify.
//   Q3. What does the fused covariance look like after a perfect linear
//       observation (correlation → 1)?  Why is that degenerate?
//
// ── What you must implement ──────────────────────────────────────────────────
//
// class Covariance2d {
//   var_x, var_y, cov_xy   (private doubles)
//   static int instance_count_
//
//   PRIMARY constructor: Covariance2d(double var_x, double var_y, double cov_xy)
//     - Validates: var_x > 0, then var_x*var_y - cov_xy^2 > 0.
//     - Throw std::invalid_argument with a message naming the violated condition.
//     - Increments instance_count_.
//
//   static factory: identity()
//     - Returns Covariance2d(1, 1, 0).  No extra logic.
//
//   static factory: from_std_devs(double sx, double sy, double rho)
//     - rho is the Pearson correlation coefficient, must be in (-1, 1) exclusive.
//     - Converts to matrix form and delegates to the primary constructor.
//     - THINK: cov_xy = rho * sx * sy.  Substitute into the PD condition.
//       What does |rho| < 1 guarantee about the determinant?
//     - If rho is outside (-1, 1) throw std::invalid_argument BEFORE
//       constructing anything.
//
//   explicit single-arg constructor: Covariance2d(double var)
//     - Represents an isotropic (spherical) covariance: var_x=var_y=var, cov_xy=0.
//     - Must delegate to the primary constructor.
//     - Mark explicit so Covariance2d c = 3.0; is a compile error.
//
//   Destructor: decrements instance_count_.
//
//   Accessors: var_x(), var_y(), cov_xy()   — const, return the stored values.
//
//   double correlation() const
//     - Returns cov_xy / sqrt(var_x * var_y).
//     - THINK: can this value ever exceed 1 or be below -1 in a valid object?
//
//   bool is_positive_definite() const
//     - Re-checks the conditions.  Should always return true for a live object —
//       but write it generically so it could be used on any 3-double bundle.
//
//   bool operator==(const Covariance2d& other) const
//     - All three fields match within epsilon = 1e-9.
//
//   static int active_count()
//     - Returns instance_count_.
// };
//
// ── Static member definition ─────────────────────────────────────────────────
// Declare inside the class, define (= 0) OUTSIDE.  If you define it inside you
// will get a linker error with multiple translation units.  Why?
//
// ── Delegating constructor chain ─────────────────────────────────────────────
// from_std_devs is NOT a constructor — it is a static factory returning a value.
// The single-arg constructor Covariance2d(double var) IS a constructor and must
// delegate:  Covariance2d(var, var, 0.0)  in the member-initializer list.
// There is one subtle ordering issue: member initializers run BEFORE the body,
// and a delegating constructor transfers ALL initialization to the target.
// You cannot mix delegation with member initializers.
//
// ── Hard edge case ───────────────────────────────────────────────────────────
// What if the caller passes var_x=1e-20 and cov_xy=1e-10?
//   var_x * var_y - cov_xy^2 = 1e-40 - 1e-20 < 0   → not PD, must throw.
// The naive check (var_x > 0 && var_y > 0) would pass; the determinant check
// catches it.  Make sure your validation runs in the right order.
//
// ── Expected output ──────────────────────────────────────────────────────────
//   identity: var_x=1 var_y=1 cov_xy=0 corr=0
//   custom: var_x=4 var_y=9 cov_xy=3 corr=0.5
//   identity is PD: 1
//   active: 2
//   [exception] from_std_devs with corr=1.0: correlation must be in (-1, 1)
//   [exception] direct with negative var: var_x must be positive
//   active: 2
//   active after scope: 0

#include <iostream>
#include <stdexcept>
#include <cmath>
#include <string>

// ── YOUR ANSWERS (fill in before coding) ────────────────────────────────────
// A1 (why rho=±1 breaks PD):
//   TODO: answer here
//
// A2 (var_x=4, var_y=9, cov_xy=6, is it PD?):
//   TODO: compute det = var_x*var_y - cov_xy^2 = ... answer here
//
// A3 (perfect observation, rho→1):
//   TODO: answer here

// ── class Covariance2d ───────────────────────────────────────────────────────

class Covariance2d {
public:
    // TODO: declare static int instance_count_
    //   (definition goes outside the class — see below)

    // TODO: PRIMARY constructor Covariance2d(double var_x, double var_y, double cov_xy)
    //   Step 1 — validate var_x > 0, throw std::invalid_argument("var_x must be positive") if not.
    //   Step 2 — validate var_x*var_y - cov_xy*cov_xy > 0,
    //             throw std::invalid_argument("matrix is not positive-definite") if not.
    //   Step 3 — store and increment counter.
    //   TRICKY: do NOT store before validating; a half-constructed object that
    //           increments the counter before throwing leaves a stale count.
    //           Increment ONLY after all checks pass.

    // TODO: explicit single-arg constructor Covariance2d(double var)
    //   Must delegate to the primary constructor: Covariance2d(var, var, 0.0)
    //   The `explicit` keyword prevents implicit conversions like:
    //     void f(Covariance2d); f(3.0);  // should be a compile error

    // TODO: Destructor — decrement instance_count_

    // TODO: static factory identity()
    //   Returns Covariance2d(1.0, 1.0, 0.0)

    // TODO: static factory from_std_devs(double sx, double sy, double rho)
    //   Validate rho in (-1, 1) exclusive FIRST.
    //   Then return Covariance2d(sx*sx, sy*sy, rho*sx*sy).
    //   THINK: after substitution, the PD condition becomes:
    //     sx^2 * sy^2 - (rho*sx*sy)^2 = sx^2*sy^2*(1 - rho^2)
    //   This is > 0 iff |rho| < 1 and sx,sy > 0.  So the rho check here
    //   makes the PD check in the primary constructor redundant — but both
    //   should still fire to protect against callers bypassing from_std_devs.

    // TODO: accessors var_x(), var_y(), cov_xy() — return the private members

    // TODO: double correlation() const
    //   cov_xy_ / std::sqrt(var_x_ * var_y_)
    //   Guaranteed to be in (-1,1) for any valid Covariance2d object.

    // TODO: bool is_positive_definite() const
    //   Re-check the two SPD conditions.  A valid live object always passes,
    //   but this is useful for external callers who hold raw values.

    // TODO: bool operator==(const Covariance2d& other) const
    //   Use std::abs(a - b) < 1e-9 for each field.

    static int active_count();   // returns instance_count_

private:
    double var_x_;
    double var_y_;
    double cov_xy_;
};

// TODO: define Covariance2d::instance_count_ here (outside class, initialise to 0)
//   int Covariance2d::instance_count_ = 0;

// TODO: implement active_count() here (or inline in the class — your choice,
//   but think about which is better style and why)


// ── main ─────────────────────────────────────────────────────────────────────
int main() {
    {
        auto id  = Covariance2d::identity();
        auto cov = Covariance2d::from_std_devs(2.0, 3.0, 0.5);  // sx=2,sy=3,rho=0.5

        std::cout << "identity: var_x=" << id.var_x()
                  << " var_y="  << id.var_y()
                  << " cov_xy=" << id.cov_xy()
                  << " corr="   << id.correlation() << "\n";

        std::cout << "custom: var_x=" << cov.var_x()
                  << " var_y="  << cov.var_y()
                  << " cov_xy=" << cov.cov_xy()
                  << " corr="   << cov.correlation() << "\n";

        std::cout << "identity is PD: " << id.is_positive_definite() << "\n";
        std::cout << "active: " << Covariance2d::active_count() << "\n";

        // Exception: correlation = 1.0 (singular matrix)
        try {
            auto bad = Covariance2d::from_std_devs(2.0, 3.0, 1.0);
            (void)bad;
        } catch (const std::invalid_argument& e) {
            std::cout << "[exception] from_std_devs with corr=1.0: " << e.what() << "\n";
        }

        // Exception: negative variance
        try {
            Covariance2d bad(-1.0, 1.0, 0.0);
            (void)bad;
        } catch (const std::invalid_argument& e) {
            std::cout << "[exception] direct with negative var: " << e.what() << "\n";
        }

        // Counter must still be 2 — the failed constructions must NOT
        // have incremented it.
        std::cout << "active: " << Covariance2d::active_count() << "\n";
    } // id and cov destroyed here

    std::cout << "active after scope: " << Covariance2d::active_count() << "\n";
    return 0;
}
