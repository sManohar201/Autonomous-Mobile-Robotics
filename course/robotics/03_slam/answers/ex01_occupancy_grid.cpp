#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

struct Grid {
    int width, height;
    double resolution;
    std::vector<double> cells;

    Grid(int w, int h, double res)
        : width(w), height(h), resolution(res), cells(w * h, 0.0) {}

    double& at(int x, int y) { return cells[y * width + x]; }
    const double& at(int x, int y) const { return cells[y * width + x]; }

    bool in_bounds(int x, int y) const {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    double probability(int x, int y) const {
        return 1.0 / (1.0 + std::exp(-at(x, y)));
    }
};

struct Pose { double x, y, theta; };

void integrate_scan(Grid& grid, const Pose& robot,
                    const std::vector<double>& ranges,
                    double angle_min, double angle_inc) {
    const double LOG_FREE = -0.4, LOG_OCC = 0.9;

    for (size_t i = 0; i < ranges.size(); ++i) {
        if (ranges[i] <= 0) continue;
        const double angle = robot.theta + angle_min + i * angle_inc;
        const double dist  = ranges[i];

        for (double r = grid.resolution; r < dist - grid.resolution;
             r += grid.resolution) {
            int cx = int((robot.x + r * std::cos(angle)) / grid.resolution);
            int cy = int((robot.y + r * std::sin(angle)) / grid.resolution);
            if (grid.in_bounds(cx, cy))
                grid.at(cx, cy) = std::clamp(grid.at(cx,cy) + LOG_FREE, -5.0, 5.0);
        }

        int ex = int((robot.x + dist * std::cos(angle)) / grid.resolution);
        int ey = int((robot.y + dist * std::sin(angle)) / grid.resolution);
        if (grid.in_bounds(ex, ey))
            grid.at(ex, ey) = std::clamp(grid.at(ex,ey) + LOG_OCC, -5.0, 5.0);
    }
}

int main() {
    Grid grid(20, 20, 0.5);
    Pose robot{5.0, 5.0, 0.0};

    std::vector<double> ranges(10, 8.0);
    ranges[4] = 3.0;  // beam 4 hits wall at 3m

    const double angle_min = -M_PI / 4;
    const double angle_inc = M_PI / 20;
    integrate_scan(grid, robot, ranges, angle_min, angle_inc);

    // Compute exact endpoint cell for beam 4
    const double a4 = robot.theta + angle_min + 4 * angle_inc;
    const int wx = int((robot.x + 3.0 * std::cos(a4)) / grid.resolution);
    const int wy = int((robot.y + 3.0 * std::sin(a4)) / grid.resolution);
    const double p = grid.probability(wx, wy);

    std::cout << "Endpoint probability: " << p << "\n";
    std::cout << "(should be > 0.5 = occupied)\n";

    if (p <= 0.5) {
        std::cerr << "FAIL: endpoint should be occupied\n";
        return 1;
    }

    std::cout << "ex01_occupancy_grid passed\n";
    return 0;
}
