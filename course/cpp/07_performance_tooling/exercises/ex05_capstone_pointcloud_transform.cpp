#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

struct Point2f { float x, y; };

// Exercise: implement transform_points(points, tx, ty, theta).
//
// Apply a 2D rigid-body transform to every point:
//   x' = cos(theta)*x - sin(theta)*y + tx
//   y' = sin(theta)*x + cos(theta)*y + ty
//
// Performance requirements:
//   - Compute cos/sin once before the loop (not inside it)
//   - Reserve the output vector before pushing
//   - Do not modify the input

int main() {
    // TODO
    std::cout << "ex05_capstone_pointcloud_transform passed\n";
}
