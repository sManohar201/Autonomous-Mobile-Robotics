// Exercise 02 — Copy & Move Semantics (Rule of Five)
//
// EXPECTED OUTPUT:
//   created PointCloud(3 points)
//   created PointCloud(3 points)
//   copy-assigned 3 points
//   moved PointCloud(3 points)
//   move-assigned 3 points
//   first point of b: (1.0, 2.0, 3.0)
//   first point of d: (1.0, 2.0, 3.0)
//   destroyed PointCloud
//   destroyed PointCloud
//   destroyed PointCloud
//   destroyed PointCloud

#include <iostream>
#include <cstring>
#include <utility>

class PointCloud {
public:
    float* data;
    int    n_points;

    PointCloud(int n, const float* src) : n_points(n) {
        data = new float[n * 3];
        std::memcpy(data, src, n * 3 * sizeof(float));
        std::cout << "created PointCloud(" << n << " points)\n";
    }

    ~PointCloud() {
        delete[] data;
        std::cout << "destroyed PointCloud\n";
    }

    PointCloud(const PointCloud& other) : n_points(other.n_points) {
        data = new float[n_points * 3];
        std::memcpy(data, other.data, n_points * 3 * sizeof(float));
        std::cout << "copied PointCloud(" << n_points << " points)\n";
    }

    PointCloud& operator=(const PointCloud& other) {
        if (this == &other) return *this;
        delete[] data;
        n_points = other.n_points;
        data = new float[n_points * 3];
        std::memcpy(data, other.data, n_points * 3 * sizeof(float));
        std::cout << "copy-assigned " << n_points << " points\n";
        return *this;
    }

    PointCloud(PointCloud&& other) noexcept
        : data(other.data), n_points(other.n_points)
    {
        other.data = nullptr;
        other.n_points = 0;
        std::cout << "moved PointCloud(" << n_points << " points)\n";
    }

    PointCloud& operator=(PointCloud&& other) noexcept {
        if (this == &other) return *this;
        delete[] data;
        data = other.data;
        n_points = other.n_points;
        other.data = nullptr;
        other.n_points = 0;
        std::cout << "move-assigned " << n_points << " points\n";
        return *this;
    }
};

int main() {
    float pts[9] = {1,2,3, 4,5,6, 7,8,9};

    PointCloud a(3, pts);
    PointCloud b(3, pts);

    b = a;
    PointCloud c(std::move(a));
    b = std::move(c);

    std::cout << "first point of b: ("
              << b.data[0] << ", " << b.data[1] << ", " << b.data[2] << ")\n";

    PointCloud d(3, pts);
    d = b;
    std::cout << "first point of d: ("
              << d.data[0] << ", " << d.data[1] << ", " << d.data[2] << ")\n";

    return 0;
}
