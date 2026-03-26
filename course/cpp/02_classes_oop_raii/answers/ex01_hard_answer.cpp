// Exercise 01 — Invariant-enforcing class design: Covariance2d
// ANSWER FILE
//
// ── Research Questions ───────────────────────────────────────────────────────
//
// A1 (why rho=±1 breaks positive-definite):
//   When rho = ±1, the cross-covariance is cov_xy = rho*sx*sy = ±sx*sy.
//   The determinant is:
//     det = var_x*var_y - cov_xy^2
//         = sx^2 * sy^2 - (rho*sx*sy)^2
//         = sx^2 * sy^2 * (1 - rho^2)
//         = sx^2 * sy^2 * (1 - 1) = 0
//   A zero determinant means the matrix is singular (not invertible), so
//   it fails the positive-definite condition (det must be strictly > 0).
//   Geometrically, the uncertainty ellipse degenerates to a line — there is
//   infinite certainty in one linear combination of the state variables and
//   infinite (or zero) uncertainty in the perpendicular direction.
//
// A2 (var_x=4, var_y=9, cov_xy=6 — is it positive-definite?):
//   det = var_x*var_y - cov_xy^2 = 4*9 - 6^2 = 36 - 36 = 0.
//   NOT positive-definite (det must be strictly > 0).
//   This corresponds to rho = cov_xy / sqrt(var_x*var_y)
//                           = 6 / sqrt(36) = 6/6 = 1.0 (perfect correlation).
//   The constructor will throw "matrix is not positive-definite".
//
// A3 (fused covariance after a perfect linear observation, rho → 1):
//   When a perfect linear observation drives the correlation to ±1, the
//   updated covariance matrix becomes singular (rank-deficient, det → 0).
//   The EKF update step requires inverting the innovation covariance S = H*P*H^T + R.
//   If P is singular, S can be singular and cannot be inverted, so the Kalman
//   gain K = P*H^T * S^{-1} is undefined.  Future prediction steps propagating
//   a singular P are also undefined.
//   Practically, this means uncertainty has collapsed onto a 1-D manifold: the
//   filter "knows" a linear combination of the states perfectly and cannot
//   represent further updates without numerical instability or division by zero.

#include <iostream>
#include <stdexcept>
#include <cmath>
#include <string>

// ── class Covariance2d ───────────────────────────────────────────────────────

class Covariance2d {
public:
    // Static instance counter — declared here, defined outside the class.
    // Must be defined outside because only one definition may exist across all
    // translation units.  An inline definition inside the class body (without
    // inline specifier pre-C++17) would violate the One Definition Rule when
    // the header is included in multiple .cpp files.
    static int instance_count_;

    // PRIMARY constructor — validates invariants before touching member state
    // or the counter.  Throwing before ++instance_count_ means a failed
    // construction never inflates the count.
    Covariance2d(double var_x, double var_y, double cov_xy) {
        // Step 1: first principal minor
        if (var_x <= 0.0)
            throw std::invalid_argument("var_x must be positive");
        // Step 2: determinant (second principal minor)
        if (var_x * var_y - cov_xy * cov_xy <= 0.0)
            throw std::invalid_argument("matrix is not positive-definite");
        // Step 3: store and count only after all validation passes
        var_x_  = var_x;
        var_y_  = var_y;
        cov_xy_ = cov_xy;
        ++instance_count_;
    }

    // Single-argument convenience constructor for isotropic (spherical) covariance.
    // Marked explicit to prevent silent implicit conversions such as:
    //   void f(Covariance2d); f(3.0);   // compile error with explicit
    // Delegates to the primary constructor; member initializers are not allowed
    // when delegating — all initialisation is transferred to the target.
    explicit Covariance2d(double var) : Covariance2d(var, var, 0.0) {}

    // Destructor decrements the count exactly once per successfully constructed
    // object.  Because the counter is only incremented after validation, every
    // live object has a matching decrement here.
    ~Covariance2d() { --instance_count_; }

    // Factory: returns the 2D identity covariance (unit variance, no correlation).
    static Covariance2d identity() {
        return Covariance2d(1.0, 1.0, 0.0);
    }

    // Factory: constructs from standard deviations and Pearson correlation rho.
    // Validates rho ∈ (-1, 1) before delegating to the primary constructor.
    // The PD condition after substitution becomes:
    //   sx^2 * sy^2 - (rho*sx*sy)^2 = sx^2*sy^2*(1 - rho^2) > 0
    // which holds iff |rho| < 1 and sx, sy > 0.  The rho guard here is a
    // domain-level check; the primary constructor still enforces the algebraic
    // condition for callers who bypass this factory.
    static Covariance2d from_std_devs(double sx, double sy, double rho) {
        if (rho <= -1.0 || rho >= 1.0)
            throw std::invalid_argument("correlation must be in (-1, 1)");
        return Covariance2d(sx * sx, sy * sy, rho * sx * sy);
    }

    // Accessors
    double var_x()  const { return var_x_;  }
    double var_y()  const { return var_y_;  }
    double cov_xy() const { return cov_xy_; }

    // Pearson correlation coefficient: guaranteed in (-1, 1) for any valid object.
    double correlation() const {
        return cov_xy_ / std::sqrt(var_x_ * var_y_);
    }

    // Re-validates the SPD conditions.  For any successfully constructed
    // Covariance2d this will always return true, but it is useful as a
    // general-purpose check on raw values.
    bool is_positive_definite() const {
        return var_x_ > 0.0 && (var_x_ * var_y_ - cov_xy_ * cov_xy_) > 0.0;
    }

    // Equality within floating-point epsilon.
    bool operator==(const Covariance2d& other) const {
        return std::abs(var_x_  - other.var_x_)  < 1e-9 &&
               std::abs(var_y_  - other.var_y_)  < 1e-9 &&
               std::abs(cov_xy_ - other.cov_xy_) < 1e-9;
    }

    // Returns the number of currently live Covariance2d objects.
    static int active_count() { return instance_count_; }

private:
    double var_x_;
    double var_y_;
    double cov_xy_;
};

// Out-of-class definition of the static data member.
// This is the one-and-only definition; the declaration inside the class is
// just a declaration.  Without this line the linker reports an undefined symbol.
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
