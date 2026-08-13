# 3. A* on a Grid — C++ Implementation

```cpp
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <vector>

struct Cell { int x, y; };
struct Node { Cell cell; double f; };
bool operator>(const Node& a, const Node& b) { return a.f > b.f; }

using Grid = std::vector<std::vector<int>>;  // 0=free, 1=obstacle

double heuristic(const Cell& a, const Cell& b) {
    return std::sqrt(std::pow(a.x - b.x, 2.0) + std::pow(a.y - b.y, 2.0));
}

std::vector<Cell> astar(const Grid& grid, Cell start, Cell goal) {
    const int rows = grid.size(), cols = grid[0].size();
    auto in_bounds = [&](int x, int y) {
        return x >= 0 && x < cols && y >= 0 && y < rows;
    };
    auto is_free = [&](int x, int y) { return grid[y][x] == 0; };

    // g cost and parent tracking
    std::vector<std::vector<double>> g(rows, std::vector<double>(cols, 1e9));
    std::unordered_map<int, Cell> parent;
    auto key = [&](int x, int y) { return y * cols + x; };

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    g[start.y][start.x] = 0.0;
    open.push({start, heuristic(start, goal)});

    // 8-connected neighbours
    const int dx[] = {-1,0,1,-1,1,-1,0,1};
    const int dy[] = {-1,-1,-1,0,0,1,1,1};
    const double dc[] = {1.414,1,1.414,1,1,1.414,1,1.414};

    while (!open.empty()) {
        auto [cur, _] = open.top(); open.pop();
        if (cur.x == goal.x && cur.y == goal.y) break;

        for (int i = 0; i < 8; ++i) {
            int nx = cur.x + dx[i], ny = cur.y + dy[i];
            if (!in_bounds(nx, ny) || !is_free(nx, ny)) continue;
            double g_new = g[cur.y][cur.x] + dc[i];
            if (g_new < g[ny][nx]) {
                g[ny][nx] = g_new;
                parent[key(nx, ny)] = cur;
                open.push({{nx, ny}, g_new + heuristic({nx, ny}, goal)});
            }
        }
    }

    // Reconstruct path
    std::vector<Cell> path;
    Cell c = goal;
    while (!(c.x == start.x && c.y == start.y)) {
        path.push_back(c);
        auto it = parent.find(key(c.x, c.y));
        if (it == parent.end()) return {};  // no path
        c = it->second;
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

int main() {
    // 5x5 grid with a wall
    Grid grid = {
        {0,0,0,0,0},
        {0,1,1,1,0},
        {0,1,0,0,0},
        {0,0,0,1,0},
        {0,0,0,0,0},
    };
    auto path = astar(grid, {0,0}, {4,4});
    std::cout << "Path length: " << path.size() << " cells\n";
    for (const auto& [x, y] : path)
        std::cout << "  (" << x << "," << y << ")\n";
    return 0;
}
```
