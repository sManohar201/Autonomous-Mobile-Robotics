// Moderate Exercise — A* Path Planning on a 2D Grid
//
// Implement A* to find the shortest path from start to goal on a binary grid.
//   Grid: 0 = free, 1 = obstacle
//   Connectivity: 8-connected (diagonal moves cost √2 ≈ 1.414)
//
// Algorithm:
//   f(n) = g(n) + h(n)
//   g(n) = actual cost from start to n
//   h(n) = Euclidean distance from n to goal (admissible heuristic)
//
// Steps:
//   1. Use a min-heap (priority_queue with greater<>) sorted by f
//   2. Track g cost and parent cell for each node
//   3. When goal is popped, reconstruct path by following parents back to start
//   4. Return empty vector if no path exists
//
// Hints:
//   - std::priority_queue<Node, vector<Node>, greater<Node>> for min-heap
//   - 2D g-cost grid: vector<vector<double>> initialised to 1e9
//   - Parent map: unordered_map<int, Cell> where key = y*cols + x
//   - 8 directions: {-1,-1},{0,-1},{1,-1},{-1,0},{1,0},{-1,1},{0,1},{1,1}
//   - Diagonal cost: 1.414, axis-aligned cost: 1.0

#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <vector>

struct Cell { int x, y; };

// TODO: implement astar(grid, start, goal) -> vector<Cell>

int main() {
    std::vector<std::vector<int>> grid = {
        {0,0,0,0,0},
        {0,1,1,1,0},
        {0,1,0,0,0},
        {0,0,0,1,0},
        {0,0,0,0,0},
    };

    // TODO: call astar and print path
    // Expected: path from (0,0) to (4,4), length > 0, no cells in obstacles

    std::cout << "ex01_astar passed\n";
    return 0;
}
