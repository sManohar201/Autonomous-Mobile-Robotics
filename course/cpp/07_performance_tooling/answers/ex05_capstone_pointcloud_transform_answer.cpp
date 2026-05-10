#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

struct Point2f {
    float x, y;
};

std::vector<Point2f> transform_points(const std::vector<Point2f>& points,
                                      float tx,
                                      float ty,
                                      float theta) {
    const float c = std::cos(theta);
    const float s = std::sin(theta);
    std::vector<Point2f> out;
    out.reserve(points.size());
    for (const Point2f& p : points) {
        out.push_back(Point2f{c * p.x - s * p.y + tx,
                              s * p.x + c * p.y + ty});
    }
    return out;
}

int main() {
    std::vector<Point2f> points{{1.0f, 0.0f}};
    auto out = transform_points(points, 1.0f, 2.0f, 0.0f);
    assert(out.size() == 1);
    assert(out[0].x == 2.0f);
    assert(out[0].y == 2.0f);
    std::cout << "ex05_capstone_pointcloud_transform_answer passed\n";
}

