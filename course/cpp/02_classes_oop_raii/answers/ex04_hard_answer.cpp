// Exercise 04 (Hard) — SE(2) Rigid Body Transforms
// ANSWER FILE
//
// ── Research question answers ────────────────────────────────────────────────
//
// Q1. Non-commutativity of SE(2) composition:
//     Let T1 = (tx=1, ty=0, θ=π/2) and T2 = (tx=0, ty=1, θ=0).
//
//     T1 ∘ T2:
//       theta = π/2 + 0                                    = π/2
//       tx    = 1 + cos(π/2)*0 - sin(π/2)*1  = 1 + 0 - 1  = 0
//       ty    = 0 + sin(π/2)*0 + cos(π/2)*1  = 0 + 0 + 0  = 0
//     → T1 ∘ T2 = (0, 0, π/2)
//
//     T2 ∘ T1:
//       theta = 0 + π/2                                    = π/2
//       tx    = 0 + cos(0)*1 - sin(0)*0  = 0 + 1 - 0      = 1
//       ty    = 1 + sin(0)*1 + cos(0)*0  = 1 + 0 + 0      = 1
//     → T2 ∘ T1 = (1, 1, π/2)
//
//     They differ: (0,0,π/2) ≠ (1,1,π/2).
//     Intuition: T1 first rotates the frame 90° and THEN places T2's translation
//     into that rotated frame.  T2 first translates along y (in the original
//     frame), THEN rotates.  The rotation mixes tx2 and ty2 differently depending
//     on which transform is on the left.  SE(2) group composition is not
//     commutative in general.
//
// Q2. Inverse of T1 = (1, 0, π/2):
//     Formula: inv.theta = -θ
//              inv.tx    = -(cos(θ)*tx + sin(θ)*ty)
//              inv.ty    =   sin(θ)*tx - cos(θ)*ty
//
//     inv.theta = -π/2
//     inv.tx    = -(cos(π/2)*1 + sin(π/2)*0) = -(0 + 0)   = 0
//     inv.ty    =   sin(π/2)*1 - cos(π/2)*0  = 1 - 0       = 1
//     → T1_inv = (0, 1, -π/2)
//
//     Verification — T1 * T1_inv should equal identity (0, 0, 0):
//       theta = π/2 + (-π/2) = 0   ✓
//       tx    = 1 + cos(π/2)*0 - sin(π/2)*1  = 1 + 0 - 1  = 0   ✓
//       ty    = 0 + sin(π/2)*0 + cos(π/2)*1  = 0 + 0 + 0  = 0   ✓
//
// Q3. T1 applied to p = (1, 0):
//     new_x = tx + cos(θ)*px - sin(θ)*py = 1 + cos(π/2)*1 - sin(π/2)*0
//           = 1 + 0 - 0 ≈ 1   (cos(π/2) is ~6e-17 in floating point)
//     new_y = ty + sin(θ)*px + cos(θ)*py = 0 + sin(π/2)*1 + cos(π/2)*0
//           = 0 + 1 + 0 = 1
//     → T1(1, 0) ≈ (1, 1)   ✓  (exact to 2 decimal places)
//
// ── operator*= trap ──────────────────────────────────────────────────────────
// A naive in-place computation of T1 *= T2 would write:
//   tx_    = tx_ + cos(theta_)*other.tx_ - sin(theta_)*other.ty_;
//   ty_    = ty_ + sin(theta_)*other.tx_ + cos(theta_)*other.ty_;  // WRONG: tx_ already overwritten
//   theta_ = theta_ + other.theta_;
// The new ty_ uses the old tx_ implicitly through cos(theta_), but tx_ was
// already overwritten.  The fix is to delegate to operator* which builds a
// fresh Transform2d, then assign the result to *this.

#include <iostream>
#include <iomanip>
#include <cmath>
#include <cassert>

// ── Vec2 ─────────────────────────────────────────────────────────────────────

struct Vec2 {
    double x, y;
    Vec2(double x = 0.0, double y = 0.0) : x(x), y(y) {}
};

std::ostream& operator<<(std::ostream& os, const Vec2& v) {
    os << std::fixed << std::setprecision(2)
       << "(" << v.x << ", " << v.y << ")";
    return os;
}

// ── Transform2d ──────────────────────────────────────────────────────────────

class Transform2d {
public:
    // Construct from translation (tx, ty) and rotation angle theta (radians).
    Transform2d(double tx, double ty, double theta)
        : tx_(tx), ty_(ty), theta_(theta) {}

    // Identity element: T * identity() == T == identity() * T for all T.
    static Transform2d identity() {
        return Transform2d(0.0, 0.0, 0.0);
    }

    // Accessors ───────────────────────────────────────────────────────────────
    double tx()    const { return tx_; }
    double ty()    const { return ty_; }
    double theta() const { return theta_; }

    // SE(2) composition — "this ∘ other", i.e. first apply other, then this.
    //
    //   (T1 ∘ T2)(p) = T1(T2(p)) = R(θ1)*(R(θ2)*p + t2) + t1
    //                = R(θ1+θ2)*p + R(θ1)*t2 + t1
    //
    // The new translation is the current translation plus the rotation of
    // other's translation by *this's angle.  This is NOT symmetric in the two
    // operands — that is the origin of non-commutativity.
    Transform2d operator*(const Transform2d& other) const {
        const double c = std::cos(theta_);
        const double s = std::sin(theta_);
        return Transform2d(
            tx_ + c * other.tx_ - s * other.ty_,
            ty_ + s * other.tx_ + c * other.ty_,
            theta_ + other.theta_
        );
    }

    // Apply transform to a 2D point: T(p) = R(θ)*p + t.
    Vec2 operator*(const Vec2& p) const {
        const double c = std::cos(theta_);
        const double s = std::sin(theta_);
        return Vec2(
            tx_ + c * p.x - s * p.y,
            ty_ + s * p.x + c * p.y
        );
    }

    // Inverse: T^{-1} = (R^T, -R^T * t).
    // Since R(θ)^T = R(-θ), rotating the negated translation by -θ gives
    // the inverse translation.
    Transform2d inverse() const {
        const double c = std::cos(theta_);
        const double s = std::sin(theta_);
        return Transform2d(
            -(c * tx_ + s * ty_),
             (s * tx_ - c * ty_),
            -theta_
        );
    }

    // Compound assignment — MUST delegate to operator* to avoid the partial
    // overwrite trap described in the header comment.
    Transform2d& operator*=(const Transform2d& other) {
        *this = *this * other;
        return *this;
    }

    // Equality within floating-point tolerance of 1e-9 per field.
    // A component-wise epsilon test is correct here because all three fields
    // (tx, ty, theta) are independent coordinates, not a norm.
    bool operator==(const Transform2d& other) const {
        return std::abs(tx_    - other.tx_)    < 1e-9 &&
               std::abs(ty_    - other.ty_)    < 1e-9 &&
               std::abs(theta_ - other.theta_) < 1e-9;
    }

    bool operator!=(const Transform2d& other) const {
        return !(*this == other);
    }

private:
    double tx_, ty_, theta_;
};

// Stream output — 4 decimal places gives enough precision for π/2 ≈ 1.5708.
std::ostream& operator<<(std::ostream& os, const Transform2d& T) {
    os << std::fixed << std::setprecision(4)
       << "(tx=" << T.tx()
       << ", ty=" << T.ty()
       << ", θ=" << T.theta() << ")";
    return os;
}

// ── main ─────────────────────────────────────────────────────────────────────
int main() {
    const double pi_2 = std::acos(-1.0) / 2.0;  // π/2

    Transform2d T1(1.0, 0.0, pi_2);   // translate x by 1, rotate 90°
    Transform2d T2(0.0, 1.0, 0.0);    // translate y by 1, no rotation
    Transform2d T3(1.0, 1.0, pi_2);   // another arbitrary transform

    std::cout << std::fixed;
    std::cout << "T1 = " << T1 << "\n";
    std::cout << "T2 = " << T2 << "\n";

    Transform2d T12 = T1 * T2;
    Transform2d T21 = T2 * T1;
    std::cout << "T1 * T2 = " << T12 << "\n";
    std::cout << "T2 * T1 = " << T21 << "\n";

    // Non-commutativity
    std::cout << "T1 * T2 == T2 * T1: " << (T12 == T21) << "\n";

    // Associativity: (T1*T2)*T3 == T1*(T2*T3)
    bool assoc = ((T1 * T2) * T3) == (T1 * (T2 * T3));
    std::cout << "(T1*T2)*T3 == T1*(T2*T3): " << assoc << "\n";
    assert(assoc && "SE(2) composition must be associative");

    // Inverse: T1 * T1^{-1} == identity
    Transform2d T1_inv = T1.inverse();
    Transform2d should_be_identity = T1 * T1_inv;
    std::cout << "T1 * T1_inv ≈ identity: "
              << (should_be_identity == Transform2d::identity()) << "\n";

    // Apply to a point
    Vec2 p(1.0, 0.0);
    Vec2 q = T1 * p;
    std::cout << "T1 applied to " << p << ": " << q << "\n";

    // Compound assignment
    Transform2d T_compound = T1;
    T_compound *= T2;
    std::cout << "T1 *= T2 result: " << T_compound << "\n";
    assert(T_compound == T12 && "operator*= must match operator*");

    return 0;
}
