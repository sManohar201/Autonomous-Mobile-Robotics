// Moderate Exercise — Occupancy Grid Builder
//
// Implement a 2D occupancy grid that integrates LiDAR range scans.
//
// Grid:
//   - Width x Height cells, each storing a log-odds value (start at 0.0)
//   - Cell (x,y) is occupied if log_odds > 0, free if < 0
//   - Convert to probability: p = 1 / (1 + exp(-log_odds))
//
// Inverse sensor model for a single beam (length dist, angle):
//   For cells along the beam (before endpoint): log_odds += LOG_FREE  (≈ -0.4)
//   For the endpoint cell: log_odds += LOG_OCC  (≈ +0.9)
//   Clamp log_odds to [-5, 5] to prevent saturation
//
// integrate_scan(grid, robot_pose, ranges, angle_min, angle_increment):
//   For each beam i:
//     global_angle = robot.theta + angle_min + i * angle_increment
//     step along beam at grid.resolution intervals, updating cells
//
// Steps:
//   1. Implement Grid struct with at(x, y), in_bounds(x, y), probability(x, y)
//   2. Implement integrate_scan
//   3. In main: create 20x20 grid at 0.5m/cell, robot at (5,5,0)
//      Simulate scan where beam 4 (index) hits a wall at 3.0m, others at 8.0m
//      After integration, check: probability(wx, wy) > 0.5 at the wall endpoint
//
// Verification: compute endpoint cell using the actual angle (angle_min + i*angle_inc),
//   not by assuming beam 4 points straight right.
// Hint: cell index for world position p: int(p / resolution)
// Hint: Use std::clamp(value, -5.0, 5.0) for clamping

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

// TODO: implement Grid struct
// TODO: implement integrate_scan

int main() {
    // TODO: create grid, define robot pose, simulate scan, check endpoint

    std::cout << "ex01_occupancy_grid passed\n";
    return 0;
}
