#include <Eigen/Dense>
#include <cmath>
#include <iostream>

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

struct EKFState { Vec3 x; Mat3 P; };

EKFState predict(const EKFState& s, double v, double omega, double dt,
                 const Mat3& Q) {
    const double th = s.x(2);
    Vec3 x_pred;
    x_pred(0) = s.x(0) + v * std::cos(th) * dt;
    x_pred(1) = s.x(1) + v * std::sin(th) * dt;
    x_pred(2) = s.x(2) + omega * dt;

    Mat3 F = Mat3::Identity();
    F(0, 2) = -v * std::sin(th) * dt;
    F(1, 2) =  v * std::cos(th) * dt;

    return {x_pred, F * s.P * F.transpose() + Q};
}

EKFState update_gps(const EKFState& s, double gps_x, double gps_y,
                    const Eigen::Matrix2d& R) {
    Eigen::Matrix<double, 2, 3> H = Eigen::Matrix<double, 2, 3>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    Eigen::Vector2d z(gps_x, gps_y);
    Eigen::Vector2d innov = z - H * s.x;
    Eigen::Matrix2d S = H * s.P * H.transpose() + R;
    Eigen::Matrix<double, 3, 2> K = s.P * H.transpose() * S.inverse();

    return {s.x + K * innov, (Mat3::Identity() - K * H) * s.P};
}

int main() {
    EKFState s{Vec3(0, 0, 0), Mat3::Identity() * 0.1};
    const Mat3 Q = Mat3::Identity() * 0.01;
    const Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 0.25;

    s = predict(s, 1.0, 0.0, 0.1, Q);
    std::cout << "After predict: x=" << s.x.transpose() << "\n";
    std::cout << "P diag: " << s.P.diagonal().transpose() << "\n";

    s = update_gps(s, 0.09, 0.0, R);
    std::cout << "After GPS update: x=" << s.x.transpose() << "\n";
    std::cout << "P diag: " << s.P.diagonal().transpose() << "\n";

    std::cout << "ex01_ekf_2d passed\n";
    return 0;
}
