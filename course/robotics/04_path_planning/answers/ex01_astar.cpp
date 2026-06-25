#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <vector>

struct Cell { int x, y; };
struct Node { Cell cell; double f; bool operator>(const Node& o) const { return f > o.f; } };

using Grid = std::vector<std::vector<int>>;

double h(const Cell& a, const Cell& b) {
    return std::sqrt(std::pow(a.x-b.x,2.0) + std::pow(a.y-b.y,2.0));
}

std::vector<Cell> astar(const Grid& grid, Cell start, Cell goal) {
    const int rows = grid.size(), cols = grid[0].size();
    auto ok = [&](int x, int y) {
        return x>=0 && x<cols && y>=0 && y<rows && grid[y][x]==0;
    };
    auto key = [&](int x, int y) { return y*cols+x; };

    std::vector<std::vector<double>> g(rows, std::vector<double>(cols, 1e9));
    std::unordered_map<int,Cell> par;
    std::priority_queue<Node,std::vector<Node>,std::greater<Node>> open;

    g[start.y][start.x] = 0;
    open.push({start, h(start, goal)});

    const int dx[]={-1,0,1,-1,1,-1,0,1}, dy[]={-1,-1,-1,0,0,1,1,1};
    const double dc[]={1.414,1,1.414,1,1,1.414,1,1.414};

    while (!open.empty()) {
        auto [cur, _] = open.top(); open.pop();
        if (cur.x==goal.x && cur.y==goal.y) break;
        for (int i=0;i<8;++i) {
            int nx=cur.x+dx[i], ny=cur.y+dy[i];
            if (!ok(nx,ny)) continue;
            double gn = g[cur.y][cur.x]+dc[i];
            if (gn < g[ny][nx]) {
                g[ny][nx]=gn; par[key(nx,ny)]=cur;
                open.push({{nx,ny}, gn+h({nx,ny},goal)});
            }
        }
    }

    std::vector<Cell> path;
    Cell c=goal;
    while (!(c.x==start.x && c.y==start.y)) {
        path.push_back(c);
        auto it=par.find(key(c.x,c.y));
        if (it==par.end()) return {};
        c=it->second;
    }
    path.push_back(start);
    std::reverse(path.begin(),path.end());
    return path;
}

int main() {
    Grid grid = {
        {0,0,0,0,0},
        {0,1,1,1,0},
        {0,1,0,0,0},
        {0,0,0,1,0},
        {0,0,0,0,0},
    };
    auto path = astar(grid, {0,0}, {4,4});
    if (path.empty()) { std::cerr << "No path!\n"; return 1; }
    std::cout << "Path (" << path.size() << " cells):\n";
    for (const auto& [x,y] : path)
        std::cout << "  (" << x << "," << y << ")\n";
    std::cout << "ex01_astar passed\n";
    return 0;
}
