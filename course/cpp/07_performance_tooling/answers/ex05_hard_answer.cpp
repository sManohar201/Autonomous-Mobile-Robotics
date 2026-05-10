#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

struct Point2f { float x, y; bool valid; };
struct Bounds { float min_x, max_x, min_y, max_y; };
struct TransformResult { std::vector<Point2f> points; Bounds bounds; };

TransformResult transform_valid(const std::vector<Point2f>& in, float tx, float ty, float theta) {
    const float c = std::cos(theta);
    const float s = std::sin(theta);
    TransformResult result{{}, {std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
                               std::numeric_limits<float>::max(), -std::numeric_limits<float>::max()}};
    result.points.reserve(in.size());
    for (const auto& p : in) {
        if (!p.valid) continue;
        Point2f q{c * p.x - s * p.y + tx, s * p.x + c * p.y + ty, true};
        result.bounds.min_x = std::min(result.bounds.min_x, q.x);
        result.bounds.max_x = std::max(result.bounds.max_x, q.x);
        result.bounds.min_y = std::min(result.bounds.min_y, q.y);
        result.bounds.max_y = std::max(result.bounds.max_y, q.y);
        result.points.push_back(q);
    }
    return result;
}

int main() {
    auto r = transform_valid({{1,0,true}, {9,9,false}}, 1, 2, 0);
    assert(r.points.size() == 1);
    assert(r.points[0].x == 2.0f);
    assert(r.bounds.min_y == 2.0f);
    std::cout << "ex05_hard_answer passed\n";
}
