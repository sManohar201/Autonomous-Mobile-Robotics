// Moderate Exercise — 2D ICP (Iterative Closest Point)
//
// Implement point-to-point ICP that aligns a source cloud to a target cloud.
//
// Algorithm (repeat until convergence):
//   1. Correspondence: for each source point, find the nearest target point
//      Hint: brute-force O(N²) is fine for small clouds
//   2. Optimal transform: compute (R, t) via SVD on the cross-covariance matrix
//      a. Compute centroids: src_mean, tgt_mean
//      b. Demean both sets
//      c. H = Σ src_demeaned[i] * tgt_demeaned[i]ᵀ
//      d. [U, S, Vᵀ] = SVD(H)
//      e. R = V * Uᵀ
//      f. t = tgt_mean - R * src_mean
//   3. Apply: transform each source point by (R, t)
//   4. Stop if RMS change < 1e-6 or max_iterations reached
//
// Return the accumulated (R, t) transform.
//
// Hint: Use Eigen::JacobiSVD with ComputeFullU | ComputeFullV
// Hint: If det(R) < 0, flip the sign of the last column of V to fix reflection.

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

using Pt2 = Eigen::Vector2d;

// TODO: implement icp_2d
// icp_2d(src, tgt, max_iter=50, tol=1e-6) -> (R 2x2, t 2d)

int main() {
    // Source: asymmetric L-shape (avoids symmetry ambiguity in correspondences)
    std::vector<Pt2> source = {{0,0},{2,0},{0,1}};
    // Target: source rotated 30° and translated (0.5, 0.3)
    // Ground truth: theta=30deg, t=(0.5, 0.3)
    std::vector<Pt2> target = {{0.500, 0.300}, {2.232, 1.300}, {0.000, 1.166}};

    // TODO: call icp_2d and print recovered rotation angle and translation

    std::cout << "ex01_icp_2d passed\n";
    return 0;
}
