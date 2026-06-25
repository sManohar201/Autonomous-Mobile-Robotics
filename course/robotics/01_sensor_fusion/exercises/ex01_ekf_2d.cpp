// Moderate Exercise — 2D EKF for a Differential Drive Robot
//
// Implement a 2D Extended Kalman Filter for a robot with state [px, py, theta].
//
// State:
//   x = [px, py, theta]   — position and heading
//
// Process model (differential drive, timestep dt):
//   px'    = px + v * cos(theta) * dt
//   py'    = py + v * sin(theta) * dt
//   theta' = theta + omega * dt
//
// Process Jacobian F = df/dx (3x3):
//   [1  0  -v*sin(theta)*dt]
//   [0  1   v*cos(theta)*dt]
//   [0  0   1              ]
//
// Measurement model: GPS observes [px, py] (no heading)
//   z = H * x,   H = [1 0 0; 0 1 0]
//
// Steps:
//   1. Implement predict(state, covariance, v, omega, dt, Q) -> (state', P')
//   2. Implement update_gps(state, covariance, gps_x, gps_y, R) -> (state', P')
//   3. Run the scenario below and print state + covariance diagonal after each step.
//
// Hints:
//   - Use Eigen::Matrix3d for 3x3 and Eigen::Vector3d for state
//   - For H (2x3): Eigen::Matrix<double, 2, 3>
//   - S = H*P*Hᵀ + R;  K = P*Hᵀ*S⁻¹;  innovation = z - H*x

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

// TODO: implement predict
// TODO: implement update_gps

int main() {
    // Initial state: origin, zero covariance
    Vec3 x(0, 0, 0);
    Mat3 P = Mat3::Identity() * 0.1;
    Mat3 Q = Mat3::Identity() * 0.01;
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 0.25;

    // Step 1: predict — move at 1 m/s for 0.1 s
    // x, P = predict(x, P, v=1.0, omega=0.0, dt=0.1, Q)

    // Step 2: GPS update at (0.09, 0.0)
    // x, P = update_gps(x, P, 0.09, 0.0, R)

    // TODO: print state and P.diagonal() after each step

    std::cout << "ex01_ekf_2d passed\n";
    return 0;
}
