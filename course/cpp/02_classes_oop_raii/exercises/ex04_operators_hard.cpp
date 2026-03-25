// Exercise 04 — SE(2) rigid body transforms with a full operator set
//
// Context
// -------
// A 2D robot moving on a flat floor has a pose described by three numbers:
//   (tx, ty) — position of the robot origin in the world frame
//   theta    — heading angle (radians), counterclockwise from +x
//
// This is an element of the Lie group SE(2) — the Special Euclidean group in
// 2D.  SE(2) is the set of all rigid (distance-preserving) transformations of
// the plane: translations and rotations, but NOT scaling or shearing.
//
// Key group properties you must demonstrate:
//
//   Non-commutativity:  T1 ∘ T2  ≠  T2 ∘ T1   in general
//     (translating then rotating ≠ rotating then translating)
//
//   Associativity:     (T1 ∘ T2) ∘ T3  ==  T1 ∘ (T2 ∘ T3)
//
//   Identity element:  T ∘ I  ==  I ∘ T  ==  T
//
//   Inverse element:   T ∘ T⁻¹  ==  I
//
// ── The math ────────────────────────────────────────────────────────────────
// A transform T = (tx, ty, θ) acts on a 2D point p = (px, py) as:
//
//   T(p) = R(θ) * p + t
//
// where R(θ) = [[cos θ, -sin θ], [sin θ, cos θ]] is the rotation matrix.
//
// Composition T1 ∘ T2 means "first apply T2, then apply T1":
//   (T1 ∘ T2)(p) = T1(T2(p))
//              = R(θ1) * (R(θ2)*p + t2) + t1
//              = R(θ1)*R(θ2)*p + R(θ1)*t2 + t1
//
// So:  composed.theta = θ1 + θ2
//      composed.tx    = tx1 + cos(θ1)*tx2 - sin(θ1)*ty2
//      composed.ty    = ty1 + sin(θ1)*tx2 + cos(θ1)*ty2
//
// Inverse of T = (R, t):  T⁻¹ = (R^T, -R^T * t)
//   R^T of R(θ) is R(-θ):
//     inv.theta = -θ
//     inv.tx    = -(cos(θ)*tx + sin(θ)*ty)   [first row of R^T times -t]
//     inv.ty    = -(-sin(θ)*tx + cos(θ)*ty)  = sin(θ)*tx - cos(θ)*ty
//
// Do NOT memorise these — derive them from first principles (R^T = R^{-1}
// for rotation matrices, and the group inverse formula).
//
// ── Before you code ──────────────────────────────────────────────────────────
//
// Q1. Let T1 = (tx=1, ty=0, θ=π/2) and T2 = (tx=0, ty=1, θ=0).
//     Compute T1 ∘ T2 by hand:
//       composed.theta = π/2
//       composed.tx    = 1 + cos(π/2)*0 - sin(π/2)*1  = 1 + 0 - 1 = 0
//       composed.ty    = 0 + sin(π/2)*0 + cos(π/2)*1  = 0 + 0 + 0 = 0
//     So T1 ∘ T2 = (0, 0, π/2).
//
//     Now compute T2 ∘ T1:
//       composed.theta = π/2
//       composed.tx    = 0 + cos(0)*1 - sin(0)*0  = 1
//       composed.ty    = 1 + sin(0)*1 + cos(0)*0  = 1
//     So T2 ∘ T1 = (1, 1, π/2).
//
//     They are different — non-commutativity confirmed.
//     Implement operator* to match these results exactly.
//
// Q2. Compute T1.inverse() by hand using the formula above.
//     Verify that T1 * T1.inverse() == identity (within 1e-9).
//
// Q3. What point does T1 map (1, 0) to?
//     T1(1,0) = (1 + cos(π/2)*1 - sin(π/2)*0, 0 + sin(π/2)*1 + cos(π/2)*0)
//             = (1 + 0, 0 + 1) = (1, 1)   [approximately — cos(π/2) ≈ 0]
//
// ── What you must implement ──────────────────────────────────────────────────
//
// struct Vec2 — simple 2D point
//   double x, y
//   operator<< : "(x, y)"
//
// class Transform2d
//   double tx_, ty_, theta_   (private)
//
//   Constructor: Transform2d(double tx, double ty, double theta)
//   static identity(): returns Transform2d(0, 0, 0)
//   Accessors: tx(), ty(), theta()
//
//   operator*(const Transform2d& other) const — SE(2) composition
//   operator*(const Vec2& p)            const — apply transform to point
//   inverse()                           const — return inverse transform
//   operator*=(const Transform2d& other)      — compound: *this = *this * other
//   operator==(const Transform2d& other) const — within 1e-9 per field
//   operator!=(const Transform2d& other) const — implemented via ==
//   operator<<  (free function)                 — "(tx=<tx>, ty=<ty>, θ=<theta>)"
//
// ── Subtle trap ──────────────────────────────────────────────────────────────
// When implementing operator*=, a naive "this->tx_ = tx_ + ..." is WRONG
// because you are overwriting tx_ before finishing the computation with the
// OLD tx_ value.  You must either compute all new values into temporaries first,
// or delegate to operator* and assign the result.
//
// ── Expected output ──────────────────────────────────────────────────────────
//   T1 = (tx=1.00, ty=0.00, θ=1.5708)
//   T2 = (tx=0.00, ty=1.00, θ=0.0000)
//   T1 * T2 = (tx=0.00, ty=0.00, θ=1.5708)
//   T2 * T1 = (tx=1.00, ty=1.00, θ=1.5708)
//   T1 * T2 == T2 * T1: 0
//   (T1*T2)*T3 == T1*(T2*T3): 1
//   T1 * T1_inv ≈ identity: 1
//   T1 applied to (1,0): approximately (1.00, 1.00)

#include <iostream>
#include <cmath>
#include <cassert>

// ── Vec2 ─────────────────────────────────────────────────────────────────────

struct Vec2 {
    double x, y;
    Vec2(double x = 0.0, double y = 0.0) : x(x), y(y) {}
};

// TODO: implement operator<< for Vec2
//   Format: "(x, y)"  with 2 decimal places.
std::ostream& operator<<(std::ostream& os, const Vec2& v);


// ── Transform2d ──────────────────────────────────────────────────────────────

class Transform2d {
public:
    // TODO: Constructor
    Transform2d(double tx, double ty, double theta);

    // TODO: static identity() — returns Transform2d(0, 0, 0)
    static Transform2d identity();

    // Accessors
    double tx()    const { return tx_; }
    double ty()    const { return ty_; }
    double theta() const { return theta_; }

    // TODO: operator*(const Transform2d&) const — SE(2) composition
    //   composed.theta = theta_ + other.theta_
    //   composed.tx    = tx_ + cos(theta_)*other.tx_ - sin(theta_)*other.ty_
    //   composed.ty    = ty_ + sin(theta_)*other.tx_ + cos(theta_)*other.ty_
    //   THINK: this is not symmetric in *this and other.  Why does order matter?
    Transform2d operator*(const Transform2d& other) const;

    // TODO: operator*(const Vec2&) const — apply transform to a point
    //   new_x = tx_ + cos(theta_)*p.x - sin(theta_)*p.y
    //   new_y = ty_ + sin(theta_)*p.x + cos(theta_)*p.y
    Vec2 operator*(const Vec2& p) const;

    // TODO: inverse() const — T^{-1} = (R^T, -R^T * t)
    //   inv.theta = -theta_
    //   inv.tx    = -(cos(theta_)*tx_ + sin(theta_)*ty_)
    //   inv.ty    =   sin(theta_)*tx_ - cos(theta_)*ty_
    //   Verify by hand before implementing: T * T.inverse() should give (0,0,0).
    Transform2d inverse() const;

    // TODO: operator*= — update *this = *this * other
    //   WARNING: do NOT do  tx_ = tx_ + cos(theta_)*other.tx_ - ...
    //   because you overwrite tx_ before using it to compute ty_.
    //   Compute the new transform fully, then assign.
    Transform2d& operator*=(const Transform2d& other);

    // TODO: operator== — all three fields within 1e-9
    bool operator==(const Transform2d& other) const;

    // TODO: operator!= — implemented in terms of operator==
    bool operator!=(const Transform2d& other) const;

private:
    double tx_, ty_, theta_;
};

// TODO: operator<< for Transform2d (free function)
//   Format: "(tx=<tx>, ty=<ty>, θ=<theta>)"  with 4 decimal places.
std::ostream& operator<<(std::ostream& os, const Transform2d& T);


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
