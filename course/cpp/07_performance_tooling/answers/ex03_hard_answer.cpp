#include <cassert>
#include <iostream>
#include <vector>

struct Point { float x, y, z; };
struct PointsSoA { std::vector<float> x, y, z; };

float sum_z_aos(const std::vector<Point>& points) {
    float sum = 0.0f;
    for (const auto& p : points) sum += p.z;
    return sum;
}

float sum_z_soa(const PointsSoA& points) {
    float sum = 0.0f;
    for (float z : points.z) sum += z;
    return sum;
}

int main() {
    std::vector<Point> aos{{0,0,1}, {0,0,2}};
    PointsSoA soa{{0,0}, {0,0}, {1,2}};
    assert(sum_z_aos(aos) == 3.0f);
    assert(sum_z_soa(soa) == 3.0f);
    std::cout << "ex03_hard_answer passed\n";
}

