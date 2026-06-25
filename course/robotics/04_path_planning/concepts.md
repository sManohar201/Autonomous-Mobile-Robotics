# Robotics Module 04 — Path Planning

## Goal

Understand the algorithms that find collision-free paths from start to goal:
A* for grid-based planning, RRT for configuration space sampling, and how Nav2's
planner stack (NavFn, Smac Planner) integrates these for real robots.

---

## 1. The Path Planning Problem

Given:
- Configuration space **C** — all possible robot states (positions, orientations)
- Obstacle region **C_obs** ⊂ **C** — states that cause collision
- Free space **C_free** = **C** \ **C_obs**
- Start **q_start** ∈ **C_free**
- Goal **q_goal** ∈ **C_free**

Find: a path τ: [0,1] → **C_free** with τ(0) = **q_start**, τ(1) = **q_goal**

For a circular robot, **C** = ℝ² and inflation of the map obstacles by the robot
radius reduces the problem to a point robot in free space.

---

## 2. A* (A-star)

A* is a best-first graph search algorithm guaranteed to find the shortest path
if the heuristic is **admissible** (never overestimates).

### Algorithm

```
Open:  priority queue sorted by f(n) = g(n) + h(n)
Closed: visited nodes

1. Add start to Open with f(start) = h(start)
2. Loop:
   a. Pop node n with lowest f from Open
   b. If n == goal: reconstruct and return path
   c. Add n to Closed
   d. For each neighbour m of n:
      - If m in Closed: skip
      - g_new = g(n) + cost(n, m)
      - If m not in Open OR g_new < g(m):
          g(m) = g_new
          f(m) = g_new + h(m)
          parent(m) = n
          Add m to Open
```

### Cost components

- `g(n)`: actual cost from start to n (sum of edge costs along best known path)
- `h(n)`: heuristic estimate of cost from n to goal
- `f(n) = g(n) + h(n)`: estimated total path cost through n

### Admissible heuristics for 2D grid

| Heuristic | Formula | Grid type |
|---|---|---|
| Manhattan | `|dx| + |dy|` | 4-connected |
| Euclidean | `√(dx²+dy²)` | Any |
| Chebyshev | `max(|dx|, |dy|)` | 8-connected |

Euclidean is always admissible and works for any connectivity.

### Dijkstra vs A*

Dijkstra: h(n) = 0 — explores all nodes equally. Complete and optimal.
A*: h(n) > 0 — prioritises nodes closer to goal. Faster but requires heuristic.

---

## 3. A* on a Grid — C++ Implementation

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

---

## 4. RRT (Rapidly-exploring Random Tree)

RRT builds a tree of reachable configurations by randomly sampling **C_free**
and growing the tree towards samples. Works in high-dimensional C-spaces where
A* is intractable.

### Algorithm

```
T = {q_start}

For iter = 1..N:
  q_rand = random sample from C (with goal bias ε)
  q_near = nearest node in T to q_rand
  q_new  = extend(q_near, q_rand, step_size)
  If collision_free(q_near, q_new):
    T.add(q_new, parent=q_near)
    If |q_new - q_goal| < goal_threshold:
      return path_to(q_new)
```

### Extend function (2D)

```cpp
Cell extend(const Cell& from, const Cell& to, double step) {
    const double dx = to.x - from.x, dy = to.y - from.y;
    const double dist = std::sqrt(dx*dx + dy*dy);
    if (dist < step) return to;
    return {int(from.x + step * dx/dist),
            int(from.y + step * dy/dist)};
}
```

### Goal bias

Without bias, RRT explores uniformly — slow to reach a specific goal.
With goal bias ε (typically 5–10%), sample the goal with probability ε:

```cpp
Cell sample_with_bias(double bias, Cell goal, int width, int height) {
    if (uniform(rng) < bias) return goal;
    return {uniform_int(0, width-1), uniform_int(0, height-1)};
}
```

### RRT* (asymptotically optimal)

RRT finds any valid path. RRT* adds:
1. **Near-set rewiring:** after adding q_new, check all nodes within radius r
   if routing them through q_new shortens their path.
2. **Result:** as N → ∞, the tree converges to the optimal path.

```
After adding q_new:
  For each q_near in Ball(q_new, r):
    If g(q_new) + cost(q_new, q_near) < g(q_near):
      rewire(q_near, parent=q_new)
```

### When to use RRT vs A*

| Property | A* | RRT |
|---|---|---|
| Completeness | Yes (for finite grid) | Probabilistically complete |
| Optimality | Yes (admissible h) | No (RRT*: asymptotically) |
| C-space | Low-dim, grid | Any dimension |
| Robot | Point or inflated | Full kinematic model |
| Speed | Fast for small grids | Fast for high-dim |
| Used in | Nav2 NavFn | MoveIt (robot arms), RRT* for drones |

---

## 5. Potential Fields

A simple reactive planner: the robot follows a gradient field.

- Attractive field toward goal: `F_att = -k_att · (x - x_goal)`
- Repulsive field from obstacles: `F_rep = k_rep · (1/d - 1/d₀) · (1/d²) · ∇d`
  (active only within distance d₀ of obstacle)

```cpp
Eigen::Vector2d attractive(const Eigen::Vector2d& pos,
                           const Eigen::Vector2d& goal,
                           double k = 1.0) {
    return -k * (pos - goal);
}

Eigen::Vector2d repulsive(const Eigen::Vector2d& pos,
                          const Eigen::Vector2d& obstacle,
                          double k = 0.5, double d0 = 2.0) {
    const Eigen::Vector2d diff = pos - obstacle;
    const double d = diff.norm();
    if (d > d0) return Eigen::Vector2d::Zero();
    const double factor = k * (1.0/d - 1.0/d0) * (1.0 / (d*d));
    return factor * diff.normalized();
}
```

**Local minimum problem:** the robot may get stuck in a saddle point between
attraction to goal and repulsion from obstacles. Potential fields are not
complete — they may fail even when a path exists.

---

## 6. Trajectory Optimisation

After finding a path (A* or RRT), smooth it:

### Douglas-Peucker simplification

Remove collinear waypoints within a tolerance ε:

```
1. Find point p_max with max distance to line from start to end
2. If dist(p_max) < ε: replace entire segment with straight line
3. Else: recurse on [start, p_max] and [p_max, end]
```

### Cubic spline smoothing

Fit a cubic spline through waypoints. Ensures C² continuity (smooth curvature):

```cpp
// Each segment: p(t) = a + b*t + c*t² + d*t³
// Constraints: position continuity, first and second derivative continuity
// Solve tridiagonal system for coefficients
```

Nav2's DWB controller handles smoothing implicitly by tracking a local plan.

---

## 7. Nav2 Planner Stack

Nav2's planning architecture:

```
/goal_pose (PoseStamped)
    │
    ▼
BT Navigator (behavior tree)
    │
    ▼
Planner Server
    ├── Global planner (NavFn or Smac)
    │     Input:  /map (OccupancyGrid) + costmap
    │     Output: /plan (Path)
    │
    └── Controller Server (DWB, RPP)
          Input:  /plan
          Output: /cmd_vel
```

### Global planners in Nav2

| Planner | Algorithm | Best for |
|---|---|---|
| NavFn | Dijkstra / A* | Simple environments |
| SmacPlannerHybrid | Hybrid A* | Differential/Ackermann, smooth |
| SmacPlannerLattice | State lattice | Structured environments |

### Costmaps

The 2D costmap inflates obstacle cells by the robot radius + safety margin:
- Occupied cell: 254 (lethal)
- Inflated: 1–253 (gradient cost)
- Free: 0

Inflation radius = robot_radius + safety_margin (typ. 0.3–0.5 m).

```yaml
# costmap_common_params.yaml
inflation_layer:
  plugin: nav2_costmap_2d::InflationLayer
  cost_scaling_factor: 3.0
  inflation_radius: 0.55
```

---

## 8. Path Planning in Simulation (This Repo)

```bash
# Launch simulation with Nav2
ros2 launch robot_description gazebo.launch.py world:=slam_district

# Launch Nav2 (in a second terminal)
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True

# Send a goal from RViz2: click "2D Nav Goal"
# Or from CLI:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {pose: {position: {x: 3.0, y: 2.0}}}}'
```

Monitor the planner:
```bash
ros2 topic echo /plan --field poses[0].pose.position
ros2 topic hz /cmd_vel
```

---

## 9. Interview Questions

**Q: What makes A* optimal?**
A: A* is optimal when the heuristic is admissible (never overestimates true
cost) and consistent (h(n) ≤ cost(n,m) + h(m) for all neighbours m).
Euclidean distance is both for Euclidean spaces.

**Q: What is the configuration space?**
A: C-space maps robot configurations (position, orientation, joint angles) to
a space where the robot is a point. Obstacles in workspace become C-obstacles.
This abstraction lets planners treat any robot as a point in an abstract space.

**Q: Why does RRT explore quickly?**
A: Each new node extends toward a random sample — the tree grows to cover
unexplored volume rapidly (Voronoi bias: regions with few nearby nodes get
sampled more often). This is unlike A* which follows cost gradients.

**Q: What is the local minimum problem in potential fields?**
A: The robot may get stuck where F_att + F_rep = 0 but the goal isn't reached.
This happens in concave obstacles or when the goal is directly behind an obstacle.
Fix: random walk to escape, or use wavefront propagation instead.

**Q: How does Nav2 choose between NavFn and SmacPlannerHybrid?**
A: NavFn/Dijkstra: fast, good for differential drive in open environments.
SmacHybrid: handles non-holonomic constraints (car-like) and produces
smoother paths but is slower. Use SmacHybrid for Ackermann drives or when
path smoothness matters.

**Q: What is the inflation radius in Nav2 costmaps?**
A: A buffer zone around obstacles. All cells within robot_radius are lethal —
the planner never routes through them. Cells within inflation_radius have
escalating cost — the planner avoids them when possible. This prevents the
robot's edge from touching walls even with minor localisation error.
