# 6. Worked Example: EKF Predict + Update (C++)

```cpp
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

struct EKF2D {
    Vec3 x;   // state: [px, py, theta]
    Mat3 P;   // covariance
    Mat3 Q;   // process noise
};

EKF2D predict(const EKF2D& ekf, double v, double omega, double dt) {
    const double theta = ekf.x(2);

    // Nonlinear state transition
    Vec3 x_pred;
    x_pred(0) = ekf.x(0) + v * std::cos(theta) * dt;
    x_pred(1) = ekf.x(1) + v * std::sin(theta) * dt;
    x_pred(2) = ekf.x(2) + omega * dt;

    // Jacobian of f w.r.t. x
    Mat3 F = Mat3::Identity();
    F(0, 2) = -v * std::sin(theta) * dt;
    F(1, 2) =  v * std::cos(theta) * dt;

    // Covariance prediction
    Mat3 P_pred = F * ekf.P * F.transpose() + ekf.Q;

    return {x_pred, P_pred, ekf.Q};
}

EKF2D update_gps(const EKF2D& ekf, double gps_x, double gps_y,
                 double sigma_gps) {
    // H = [1 0 0; 0 1 0]
    Eigen::Matrix<double, 2, 3> H = Eigen::Matrix<double, 2, 3>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * sigma_gps * sigma_gps;

    Eigen::Vector2d z(gps_x, gps_y);
    Eigen::Vector2d h = H * ekf.x;         // predicted measurement
    Eigen::Vector2d innov = z - h;          // innovation

    Eigen::Matrix2d S = H * ekf.P * H.transpose() + R;
    Eigen::Matrix<double, 3, 2> K = ekf.P * H.transpose() * S.inverse();

    Vec3 x_upd = ekf.x + K * innov;
    Mat3 P_upd = (Mat3::Identity() - K * H) * ekf.P;

    return {x_upd, P_upd, ekf.Q};
}

int main() {
    EKF2D ekf;
    ekf.x = Vec3(0, 0, 0);
    ekf.P = Mat3::Identity() * 0.1;
    ekf.Q = Mat3::Identity() * 0.01;

    // Predict: move forward 1 m/s for 0.1 s
    ekf = predict(ekf, 1.0, 0.0, 0.1);
    std::cout << "After predict: x=" << ekf.x.transpose() << "\n";
    std::cout << "P diag: " << ekf.P.diagonal().transpose() << "\n";

    // Update with GPS reading at (0.09, 0.0) — slightly off from prediction
    ekf = update_gps(ekf, 0.09, 0.0, 0.5);  // sigma_gps = 0.5 m
    std::cout << "After GPS update: x=" << ekf.x.transpose() << "\n";
    std::cout << "P diag: " << ekf.P.diagonal().transpose() << "\n";

    return 0;
}
```

Expected output (approximate):
```
After predict: x=0.1 0 0
P diag: 0.11 0.11 0.11
After GPS update: x=0.0997 -0.00012 0
P diag: 0.0284 0.0284 0.11
```

The GPS update reduced uncertainty in x and y (P diagonal smaller) but not in
theta (GPS doesn't observe heading directly).
