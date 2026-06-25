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
//   When rho=±1, det = var_x*var_y - (±1)^2 * var_x * var_y = var_x*var_y*(1-1) = 0.
//   The determinant equals zero, so the matrix is positive-semidefinite (singular),
//   not positive-definite. A singular covariance means one direction has zero
//   variance — the filter is claiming perfect knowledge along that axis.
//
// A2 (var_x=4, var_y=9, cov_xy=6, is it PD?):
//   det = 4*9 - 6^2 = 36 - 36 = 0. Not PD — it is only positive-semidefinite.
//   This corresponds to rho = 6/sqrt(36) = 1 — perfect correlation.
//
// A3 (perfect observation, rho→1):
//   As rho→1, the covariance matrix becomes singular (det→0). The measurement
//   update collapses the uncertainty in one direction to zero. The Kalman gain
//   becomes undefined (0/0 form) and the filter breaks numerically. In practice
//   you never achieve perfect linear correlation between state components; the
//   degeneracy signals a modelling error.

// ── class Covariance2d ───────────────────────────────────────────────────────

class Covariance2d {
public:
    static int instance_count_;

    Covariance2d(double var_x, double var_y, double cov_xy)
        : var_x_(0.0), var_y_(0.0), cov_xy_(0.0)
    {
        if (var_x <= 0.0)
            throw std::invalid_argument("var_x must be positive");
        if (var_x * var_y - cov_xy * cov_xy <= 0.0)
            throw std::invalid_argument("matrix is not positive-definite");
        var_x_  = var_x;
        var_y_  = var_y;
        cov_xy_ = cov_xy;
        ++instance_count_;
    }

    explicit Covariance2d(double var)
        : Covariance2d(var, var, 0.0)
    {}

    ~Covariance2d() { --instance_count_; }

    static Covariance2d identity() {
        return Covariance2d(1.0, 1.0, 0.0);
    }

    static Covariance2d from_std_devs(double sx, double sy, double rho) {
        if (rho <= -1.0 || rho >= 1.0)
            throw std::invalid_argument("correlation must be in (-1, 1)");
        return Covariance2d(sx * sx, sy * sy, rho * sx * sy);
    }

    double var_x()  const { return var_x_; }
    double var_y()  const { return var_y_; }
    double cov_xy() const { return cov_xy_; }

    double correlation() const {
        return cov_xy_ / std::sqrt(var_x_ * var_y_);
    }

    bool is_positive_definite() const {
        return var_x_ > 0.0 && (var_x_ * var_y_ - cov_xy_ * cov_xy_) > 0.0;
    }

    bool operator==(const Covariance2d& other) const {
        return std::abs(var_x_  - other.var_x_)  < 1e-9 &&
               std::abs(var_y_  - other.var_y_)  < 1e-9 &&
               std::abs(cov_xy_ - other.cov_xy_) < 1e-9;
    }

    static int active_count() { return instance_count_; }

private:
    double var_x_;
    double var_y_;
    double cov_xy_;
};

int Covariance2d::instance_count_ = 0;


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
