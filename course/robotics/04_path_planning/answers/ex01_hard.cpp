#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

struct Pt { int x, y; };

struct RRTNode { Pt pos; int parent; };

double dist(const Pt& a, const Pt& b) {
    return std::sqrt(std::pow(a.x-b.x,2.0) + std::pow(a.y-b.y,2.0));
}

Pt extend(const Pt& from, const Pt& to, int step) {
    const double d = dist(from, to);
    if (d <= step) return to;
    return {int(from.x + step * (to.x-from.x)/d),
            int(from.y + step * (to.y-from.y)/d)};
}

using Grid = std::vector<std::vector<int>>;

bool collision_free(const Pt& from, const Pt& to, const Grid& grid) {
    const int W = grid[0].size(), H = grid.size();
    const double d = dist(from, to);
    const int steps = std::max(1, int(d));
    for (int i=0; i<=steps; ++i) {
        int x = int(from.x + i * (to.x-from.x) / double(steps));
        int y = int(from.y + i * (to.y-from.y) / double(steps));
        if (x<0||x>=W||y<0||y>=H||grid[y][x]!=0) return false;
    }
    return true;
}

int main() {
    Grid grid(10, std::vector<int>(10, 0));
    // Add some obstacles
    grid[3][2]=grid[3][3]=grid[3][4]=1;
    grid[6][5]=grid[6][6]=grid[7][5]=1;

    const Pt start{0,0}, goal{9,9};
    const int step=2, max_iter=2000;
    const double bias=0.1;

    std::mt19937 rng(42);
    std::uniform_real_distribution<> ur(0,1);
    std::uniform_int_distribution<> rx(0,9), ry(0,9);

    std::vector<RRTNode> tree = {{start, -1}};
    int goal_idx = -1;

    for (int it=0; it<max_iter && goal_idx<0; ++it) {
        Pt q_rand = (ur(rng) < bias) ? goal : Pt{rx(rng), ry(rng)};

        int near_idx=0;
        double near_d = dist(tree[0].pos, q_rand);
        for (size_t i=1; i<tree.size(); ++i) {
            double d = dist(tree[i].pos, q_rand);
            if (d < near_d) { near_d=d; near_idx=int(i); }
        }

        Pt q_new = extend(tree[near_idx].pos, q_rand, step);
        if (!collision_free(tree[near_idx].pos, q_new, grid)) continue;

        tree.push_back({q_new, near_idx});

        if (dist(q_new, goal) <= step && collision_free(q_new, goal, grid)) {
            tree.push_back({goal, int(tree.size()-1)});
            goal_idx = int(tree.size()-1);
        }
    }

    if (goal_idx < 0) { std::cerr << "No path found\n"; return 1; }

    std::vector<Pt> path;
    for (int idx=goal_idx; idx>=0; idx=tree[idx].parent)
        path.push_back(tree[idx].pos);
    std::reverse(path.begin(), path.end());

    std::cout << "RRT path (" << path.size() << " nodes):\n";
    for (const auto& [x,y] : path)
        std::cout << "  (" << x << "," << y << ")\n";

    std::cout << "ex01_hard passed\n";
    return 0;
}
