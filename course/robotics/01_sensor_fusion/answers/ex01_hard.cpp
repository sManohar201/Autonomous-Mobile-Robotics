#include <Eigen/Dense>
#include <cmath>
#include <iostream>

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

struct EKFState { Vec3 x; Mat3 P; };

EKFState predict(const EKFState& s, double v, double omega, double dt,
                 const Mat3& Q) {
    const double th = s.x(2);
    Vec3 xp;
    xp(0) = s.x(0) + v * std::cos(th) * dt;
    xp(1) = s.x(1) + v * std::sin(th) * dt;
    xp(2) = s.x(2) + omega * dt;

    Mat3 F = Mat3::Identity();
    F(0, 2) = -v * std::sin(th) * dt;
    F(1, 2) =  v * std::cos(th) * dt;
    return {xp, F * s.P * F.transpose() + Q};
}

EKFState update_odom(const EKFState& s, const Vec3& delta, const Mat3& R) {
    const Mat3 H = Mat3::Identity();
    const Vec3 innov = delta - H * s.x;
    const Mat3 S = H * s.P * H.transpose() + R;
    const Mat3 K = s.P * H.transpose() * S.inverse();
    return {s.x + K * innov, (Mat3::Identity() - K * H) * s.P};
}

// Returns true if accepted
bool update_gps(EKFState& s, double gps_x, double gps_y,
                const Eigen::Matrix2d& R, double gate_threshold = 9.21) {
    Eigen::Matrix<double, 2, 3> H = Eigen::Matrix<double, 2, 3>::Zero();
    H(0, 0) = 1.0; H(1, 1) = 1.0;

    Eigen::Vector2d innov(gps_x - s.x(0), gps_y - s.x(1));
    Eigen::Matrix2d S = H * s.P * H.transpose() + R;
    const double maha_sq = innov.transpose() * S.inverse() * innov;

    if (maha_sq > gate_threshold) {
        std::cout << "GPS rejected: Mahalanobis²=" << maha_sq
                  << " > " << gate_threshold << "\n";
        return false;
    }

    Eigen::Matrix<double, 3, 2> K = s.P * H.transpose() * S.inverse();
    s.x = s.x + K * innov;
    s.P = (Mat3::Identity() - K * H) * s.P;
    return true;
}

int main() {
    EKFState s{Vec3(0, 0, 0), Mat3::Identity() * 0.1};
    const Mat3 Q = Mat3::Identity() * 0.005;
    const Mat3 R_odom = Mat3::Identity() * 0.01;
    const Eigen::Matrix2d R_gps = Eigen::Matrix2d::Identity() * 0.25;

    int accepted = 0, rejected = 0;

    // 5 prediction steps at 50 Hz (dt=0.02)
    for (int i = 0; i < 5; ++i) {
        s = predict(s, 1.0, 0.0, 0.02, Q);
    }

    // Odom update
    s = update_odom(s, Vec3(0.1, 0.0, 0.0), R_odom);
    std::cout << "After odom: x=" << s.x.transpose() << "\n";

    // Predict 0.1 s more
    s = predict(s, 1.0, 0.0, 0.1, Q);

    // Valid GPS
    if (update_gps(s, 0.11, 0.0, R_gps)) {
        ++accepted;
        std::cout << "GPS accepted: x=" << s.x.transpose() << "\n";
    }

    // Predict another 0.1 s
    s = predict(s, 1.0, 0.0, 0.1, Q);

    // Outlier GPS
    if (update_gps(s, 5.0, 10.0, R_gps)) {
        ++accepted;
    } else {
        ++rejected;
    }

    std::cout << "accepted=" << accepted << " rejected=" << rejected << "\n";
    std::cout << "ex01_hard passed\n";
    return 0;
}
