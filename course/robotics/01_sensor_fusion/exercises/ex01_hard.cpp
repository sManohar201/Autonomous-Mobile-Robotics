// Hard Exercise — Multi-Sensor EKF with Outlier Rejection
//
// Extend the 2D EKF from ex01_ekf_2d to fuse GPS and odometry, with
// Mahalanobis-distance outlier rejection for GPS updates.
//
// State: x = [px, py, theta]
//
// Sensors:
//   1. Odometry: provides full state delta [dpx, dpy, dtheta] at 50 Hz
//      H_odom = I_3
//   2. GPS: provides position [px, py] at 10 Hz
//      H_gps = [1 0 0; 0 1 0]
//
// Outlier rejection:
//   Compute Mahalanobis distance d² = innovᵀ * S⁻¹ * innov (S = H*P*Hᵀ + R)
//   Reject GPS update if d² > 9.21  (95% chi-squared threshold, 2 DOF)
//   Print a warning when rejecting.
//
// Run the sequence below. Print state after each update. Count accepted vs rejected.
//
// No hints — design the full implementation yourself.

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

int main() {
    // Initial state
    Vec3 x(0, 0, 0);
    Mat3 P = Mat3::Identity() * 0.1;

    // TODO: implement predict, update_odom, update_gps_with_gating

    // Fusion sequence:
    //   predict(v=1.0, omega=0.0, dt=0.02)   x5 (100ms total)
    //   update_odom([0.1, 0.0, 0.0], R_odom)
    //   predict(v=1.0, omega=0.0, dt=0.1)
    //   update_gps(0.11, 0.0, R_gps)          <- valid, accept
    //   predict(v=1.0, omega=0.0, dt=0.1)
    //   update_gps(5.0, 10.0, R_gps)          <- outlier, reject
    //   print accepted=1 rejected=1

    std::cout << "ex01_hard passed\n";
    return 0;
}
