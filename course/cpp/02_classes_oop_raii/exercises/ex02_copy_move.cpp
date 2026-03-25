// Exercise 02 — Copy & Move Semantics (Rule of Five)
//
// TASK:
//   PointCloud manages a heap-allocated array of floats (x,y,z triples).
//   Implement all five special members so it is safe to copy and move.
//   Do not change the class layout or main().
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
    // Each point is 3 floats stored flat: [x0,y0,z0, x1,y1,z1, ...]
    float* data;
    int    n_points;

    // TODO 1: Parameterised constructor.
    //         Allocates n * 3 floats on the heap, copies src into data.
    //         Prints "created PointCloud(<n> points)\n".
    PointCloud(int n, const float* src);

    // TODO 2: Destructor — release heap memory.
    //         Prints "destroyed PointCloud\n".
    ~PointCloud();

    // TODO 3: Copy constructor — deep copy.
    //         Prints "copied PointCloud(<n> points)\n".
    PointCloud(const PointCloud& other);

    // TODO 4: Copy assignment — handle self-assignment, then deep copy.
    //         Prints "copy-assigned <n> points\n".
    PointCloud& operator=(const PointCloud& other);

    // TODO 5: Move constructor — steal data pointer, leave other empty.
    //         Prints "moved PointCloud(<n> points)\n".
    //         Mark noexcept.
    PointCloud(PointCloud&& other) noexcept;

    // TODO 6: Move assignment — release own data, steal other's.
    //         Handle self-assignment. Mark noexcept.
    //         Prints "move-assigned <n> points\n".
    PointCloud& operator=(PointCloud&& other) noexcept;
};

int main() {
    float pts[9] = {1,2,3, 4,5,6, 7,8,9};

    PointCloud a(3, pts);
    PointCloud b(3, pts);

    b = a;                        // copy assignment
    PointCloud c(std::move(a));   // move constructor  (a is now empty)
    b = std::move(c);             // move assignment   (c is now empty)

    std::cout << "first point of b: ("
              << b.data[0] << ", " << b.data[1] << ", " << b.data[2] << ")\n";

    PointCloud d(3, pts);
    d = b;
    std::cout << "first point of d: ("
              << d.data[0] << ", " << d.data[1] << ", " << d.data[2] << ")\n";

    return 0;
}
