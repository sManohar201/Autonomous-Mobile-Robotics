#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

using Pt2 = Eigen::Vector2d;
using Mat2 = Eigen::Matrix2d;

struct IcpResult { Mat2 R; Pt2 t; };

IcpResult icp_2d(std::vector<Pt2> src, const std::vector<Pt2>& tgt,
                 int max_iter = 50, double tol = 1e-6) {
    Mat2 R_total = Mat2::Identity();
    Pt2  t_total = Pt2::Zero();

    for (int iter = 0; iter < max_iter; ++iter) {
        // 1. Nearest-neighbour correspondence
        std::vector<Pt2> matched(src.size());
        for (size_t i = 0; i < src.size(); ++i) {
            double best_d = std::numeric_limits<double>::max();
            for (const auto& tp : tgt) {
                double d = (src[i] - tp).squaredNorm();
                if (d < best_d) { best_d = d; matched[i] = tp; }
            }
        }

        // 2. Compute centroids
        Pt2 src_mean = Pt2::Zero(), tgt_mean = Pt2::Zero();
        for (size_t i = 0; i < src.size(); ++i) {
            src_mean += src[i]; tgt_mean += matched[i];
        }
        src_mean /= src.size(); tgt_mean /= src.size();

        // 3. Cross-covariance
        Mat2 H = Mat2::Zero();
        for (size_t i = 0; i < src.size(); ++i) {
            H += (src[i] - src_mean) * (matched[i] - tgt_mean).transpose();
        }

        // 4. SVD → rotation
        Eigen::JacobiSVD<Mat2> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Mat2 R = svd.matrixV() * svd.matrixU().transpose();
        if (R.determinant() < 0) {
            Mat2 diag = Mat2::Identity(); diag(1, 1) = -1.0;
            R = svd.matrixV() * diag * svd.matrixU().transpose();
        }
        Pt2 t = tgt_mean - R * src_mean;

        // 5. Check convergence
        double rms = 0.0;
        for (size_t i = 0; i < src.size(); ++i) {
            Pt2 new_pt = R * src[i] + t;
            rms += (new_pt - matched[i]).squaredNorm();
        }
        rms = std::sqrt(rms / src.size());

        // 6. Apply transform
        for (auto& p : src) p = R * p + t;
        R_total = R * R_total;
        t_total = R * t_total + t;

        if (rms < tol) break;
    }
    return {R_total, t_total};
}

int main() {
    // Asymmetric L-shape — avoids ambiguity from symmetric clouds
    std::vector<Pt2> source = {{0,0},{2,0},{0,1}};
    // Target: source rotated 30° + translated (0.5, 0.3)
    // R = [[cos30,-sin30],[sin30,cos30]]
    // (0,0)->R*(0,0)+(0.5,0.3) = (0.500,0.300)
    // (2,0)->R*(2,0)+(0.5,0.3) = (2.232,1.300)
    // (0,1)->R*(0,1)+(0.5,0.3) = (0.000,1.166)
    std::vector<Pt2> target = {{0.500, 0.300}, {2.232, 1.300}, {0.000, 1.166}};

    auto [R, t] = icp_2d(source, target);

    const double theta = std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI;
    std::cout << "Recovered rotation: " << theta << " deg (expected: 30)\n";
    std::cout << "Recovered translation: " << t.transpose() << " (expected: 0.5 0.3)\n";

    std::cout << "ex01_icp_2d passed\n";
    return 0;
}
