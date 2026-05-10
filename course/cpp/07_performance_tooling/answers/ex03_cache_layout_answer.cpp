#include <cassert>
#include <iostream>
#include <vector>

struct Point {
    float x, y, z;
};

float sum_z(const std::vector<Point>& points) {
    float sum = 0.0f;
    for (const Point& p : points) {
        sum += p.z;
    }
    return sum;
}

int main() {
    std::vector<Point> points{{0, 0, 1}, {0, 0, 2}, {0, 0, 3}};
    assert(sum_z(points) == 6.0f);
    std::cout << "ex03_cache_layout_answer passed\n";
}

