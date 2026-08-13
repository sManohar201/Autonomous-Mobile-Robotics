# 9. Occupancy Grid Builder — C++ Example

```cpp
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

struct Grid {
    int width, height;
    std::vector<double> cells;  // log-odds per cell
    double resolution;

    Grid(int w, int h, double res) : width(w), height(h), resolution(res),
        cells(w * h, 0.0) {}

    double& at(int x, int y) { return cells[y * width + x]; }

    bool in_bounds(int x, int y) const {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    double probability(int x, int y) const {
        return 1.0 / (1.0 + std::exp(-cells[y * width + x]));
    }
};

struct Pose { double x, y, theta; };

void integrate_scan(Grid& grid, const Pose& pose,
                    const std::vector<double>& ranges,
                    double angle_min, double angle_increment) {
    const double LOG_FREE = -0.4, LOG_OCC = 0.9;

    for (size_t i = 0; i < ranges.size(); ++i) {
        if (ranges[i] <= 0) continue;
        const double angle = pose.theta + angle_min + i * angle_increment;
        const double dist = ranges[i];

        // Mark free cells along beam
        for (double r = grid.resolution; r < dist - grid.resolution;
             r += grid.resolution) {
            int cx = int((pose.x + r * std::cos(angle)) / grid.resolution);
            int cy = int((pose.y + r * std::sin(angle)) / grid.resolution);
            if (grid.in_bounds(cx, cy))
                grid.at(cx, cy) = std::clamp(grid.at(cx,cy) + LOG_FREE, -5.0, 5.0);
        }

        // Mark endpoint occupied
        int ex = int((pose.x + dist * std::cos(angle)) / grid.resolution);
        int ey = int((pose.y + dist * std::sin(angle)) / grid.resolution);
        if (grid.in_bounds(ex, ey))
            grid.at(ex, ey) = std::clamp(grid.at(ex,ey) + LOG_OCC, -5.0, 5.0);
    }
}

int main() {
    Grid grid(20, 20, 0.5);  // 10m × 10m at 0.5m/cell
    Pose robot = {5.0, 5.0, 0.0};

    // Simulate a LiDAR with wall at 3m directly ahead
    std::vector<double> ranges(10, 0.0);
    for (int i = 0; i < 10; ++i) {
        // Only beam 4 (pointing forward) hits a wall at 3m
        ranges[i] = (i == 4) ? 3.0 : 8.0;  // other beams reach far wall
    }

    const double angle_min = -M_PI / 4;
    const double angle_inc = M_PI / 20;
    integrate_scan(grid, robot, ranges, angle_min, angle_inc);

    // Check that cell at (5+3, 5) = (8, 5) is occupied
    int wx = int((robot.x + 3.0) / grid.resolution);
    int wy = int(robot.y / grid.resolution);
    std::cout << "Endpoint probability: " << grid.probability(wx, wy) << "\n";
    std::cout << "(should be > 0.5 = occupied)\n";

    return 0;
}
```
