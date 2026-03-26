// Exercise 02 — Copy & Move Semantics (Rule of Five)
// ANSWER FILE
//
// NOTE ON EXPECTED OUTPUT:
// The exercise comment lists only 2 "created" lines and omits the "created" for
// PointCloud d(3, pts) and the "copy-assigned" for d = b.  The comment is
// incomplete.  The actual correct output from this implementation is:
//
//   created PointCloud(3 points)
//   created PointCloud(3 points)
//   copy-assigned 3 points
//   moved PointCloud(3 points)
//   move-assigned 3 points
//   first point of b: (1.0, 2.0, 3.0)
//   created PointCloud(3 points)
//   copy-assigned 3 points
//   first point of d: (1.0, 2.0, 3.0)
//   destroyed PointCloud
//   destroyed PointCloud
//   destroyed PointCloud
//   destroyed PointCloud

#include <iostream>
#include <iomanip>
#include <cstring>
#include <utility>

class PointCloud {
public:
    float* data;
    int    n_points;

    // Parameterised constructor
    PointCloud(int n, const float* src)
        : data(new float[n * 3]), n_points(n)
    {
        std::memcpy(data, src, sizeof(float) * n * 3);
        std::cout << "created PointCloud(" << n_points << " points)\n";
    }

    // Destructor
    ~PointCloud() {
        delete[] data;
        std::cout << "destroyed PointCloud\n";
    }

    // Copy constructor — deep copy
    PointCloud(const PointCloud& other)
        : data(new float[other.n_points * 3]), n_points(other.n_points)
    {
        std::memcpy(data, other.data, sizeof(float) * n_points * 3);
        std::cout << "copied PointCloud(" << n_points << " points)\n";
    }

    // Copy assignment — handle self-assignment, then deep copy
    PointCloud& operator=(const PointCloud& other) {
        if (this == &other) return *this;
        delete[] data;
        n_points = other.n_points;
        data = new float[n_points * 3];
        std::memcpy(data, other.data, sizeof(float) * n_points * 3);
        std::cout << "copy-assigned " << n_points << " points\n";
        return *this;
    }

    // Move constructor — steal data pointer, leave other empty
    PointCloud(PointCloud&& other) noexcept
        : data(other.data), n_points(other.n_points)
    {
        other.data = nullptr;
        other.n_points = 0;
        std::cout << "moved PointCloud(" << n_points << " points)\n";
    }

    // Move assignment — release own data, steal other's
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

    b = a;                        // copy assignment
    PointCloud c(std::move(a));   // move constructor  (a is now empty)
    b = std::move(c);             // move assignment   (c is now empty)

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "first point of b: ("
              << b.data[0] << ", " << b.data[1] << ", " << b.data[2] << ")\n";

    PointCloud d(3, pts);
    d = b;
    std::cout << "first point of d: ("
              << d.data[0] << ", " << d.data[1] << ", " << d.data[2] << ")\n";

    return 0;
}
