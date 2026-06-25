# Robotics Module 03 — SLAM

## Goal

Understand Simultaneous Localisation and Mapping: building a map while using that
map for localisation. Cover occupancy grid mapping, EKF SLAM with known and unknown
correspondences, loop closure detection, and graph SLAM.

---

## 1. The SLAM Problem

Localization (Module 02): given **map** m, compute p(**x**t | m, **z₁:t**)  
Mapping: given **poses** **x₁:t**, compute map m  
**SLAM**: compute p(**x₁:t**, m | **z₁:t**, **u₁:t**) simultaneously

SLAM is hard because:
- Localization needs a map
- Mapping needs accurate poses
- Both are uncertain simultaneously

```
Robot moves → uncertainty in pose grows
Robot sees landmark → pose uncertainty collapses
But landmark position is also uncertain
Seeing a known landmark again (loop closure) collapses BOTH pose and landmark uncertainty
```

---

## 2. Occupancy Grid Mapping

An occupancy grid divides 2D space into cells. Each cell stores p(occupied):

```
m(x, y) = probability that cell (x, y) is occupied
         = 0.0: definitely free
         = 0.5: unknown
         = 1.0: definitely occupied
```

### Log-odds representation

Store log-odds `l = log(p/(1-p))` instead of probability:
- Numerically stable: no probabilities near 0 or 1
- Additive update: `l_new = l_old + log_odds_update`

```cpp
double prob_to_log_odds(double p) { return std::log(p / (1.0 - p)); }
double log_odds_to_prob(double l) { return 1.0 / (1.0 + std::exp(-l)); }
```

### Inverse sensor model (2D LiDAR)

For each LiDAR beam ending at distance d:
- Cells along the beam (before endpoint): `l += log_odds_free`  (δ ≈ -0.4)
- Cell at the endpoint: `l += log_odds_occ`  (δ ≈ +0.9)
- Clamp: keep l ∈ [-5, 5] to avoid saturation

```cpp
void update_beam(Grid& grid, const Pose& robot, double angle, double dist,
                 double res) {
    const double log_free = -0.4;
    const double log_occ  =  0.9;

    // Step along beam at resolution res
    for (double r = 0; r < dist; r += res) {
        const int cx = int((robot.x + r * std::cos(angle)) / res);
        const int cy = int((robot.y + r * std::sin(angle)) / res);
        grid.at(cx, cy) = std::clamp(grid.at(cx, cy) + log_free, -5.0, 5.0);
    }
    // Mark endpoint as occupied
    const int ex = int((robot.x + dist * std::cos(angle)) / res);
    const int ey = int((robot.y + dist * std::sin(angle)) / res);
    grid.at(ex, ey) = std::clamp(grid.at(ex, ey) + log_occ, -5.0, 5.0);
}
```

### Map resolution trade-off

| Resolution | Memory (100m² map) | Accuracy | CPU |
|---|---|---|---|
| 5 cm | 40 MB | Very high | High |
| 10 cm | 10 MB | Good | Moderate |
| 20 cm | 2.5 MB | Adequate | Low |

Nav2's costmap uses 5 cm as default for navigation-grade maps.

---

## 3. EKF SLAM (Known Correspondences)

The EKF SLAM state vector augments robot pose with landmark positions:

```
x = [px, py, θ, lx1, ly1, lx2, ly2, ..., lxN, lyN]
```

For N landmarks, dimension is 3 + 2N.

### State transition

Only the robot portion of the state changes during motion:

```
F = block_diag(F_robot, I_{2N})   (landmark positions don't move)
```

The covariance update `P' = F·P·Fᵀ + Q` expands the uncertainty in the
robot-landmark cross-correlations — a key feature of EKF SLAM.

### Observation model for landmark i

Robot at (px, py, θ), landmark at (lxi, lyi):

```
expected_range   = √((lxi-px)² + (lyi-py)²)
expected_bearing = atan2(lyi-py, lxi-px) - θ

h(x) = [expected_range, expected_bearing]ᵀ
```

Jacobian Hᵢ (2 × (3+2N)) — nonzero only in robot and landmark-i columns:

```cpp
const double dx = lx - px, dy = ly - py;
const double r2 = dx*dx + dy*dy, r = std::sqrt(r2);

// Jacobian w.r.t. robot state [px, py, θ]
H.block<2,3>(0, 0) = {
    {-dx/r,  -dy/r,   0},
    { dy/r2, -dx/r2, -1}
};
// Jacobian w.r.t. landmark i state [lxi, lyi]
H.block<2,2>(0, 3 + 2*i) = {
    {dx/r,   dy/r  },
    {-dy/r2, dx/r2 }
};
```

### Adding a new landmark

When a landmark is observed for the first time, initialise it:

```cpp
state(3 + 2*n) = state(0) + range * std::cos(bearing + state(2));
state(3 + 2*n+1) = state(1) + range * std::sin(bearing + state(2));

// Augment covariance — new landmark initially uncertain
P_new = block_diag(P_old, J_init * R_landmark * J_init.transpose());
```

---

## 4. Data Association (Unknown Correspondences)

Without known correspondences, the filter must match observations to known
landmarks. Two approaches:

### Maximum likelihood (NN-based)

For each new observation z, compute Mahalanobis distance to each known landmark:

```
dᵢ² = (z - hᵢ(x))ᵀ · Sᵢ⁻¹ · (z - hᵢ(x))    where Sᵢ = Hᵢ·P·Hᵢᵀ + R
```

- If min(dᵢ²) < χ²_threshold: associate with landmark i*
- Else: create new landmark

The χ² threshold (e.g., 5.99 for 95% confidence, 2 DOF) controls the
false-positive vs. false-negative tradeoff.

### Joint compatibility branch-and-bound (JCBB)

More expensive but handles simultaneous matching of multiple landmarks.
Used in production-grade SLAM systems.

---

## 5. Loop Closure

Loop closure is the moment the robot recognises a previously visited place.
It dramatically reduces accumulated drift.

```
Without loop closure:         With loop closure:
                              
  ╭──────────╮                ╭──────────╮
 path drift  ╯               path corrected ─── closure
  ←───────                                   ↗
  start                       start ─────────
```

### Detecting loop closure

**Scan-based:** Compare current LiDAR scan to stored scans using ICP. If ICP
converges with low RMS, close the loop.

**Feature-based:** Match visual (ORB, BRIEF) or LiDAR descriptors to a
database of visited poses.

### Closing the loop

Once a loop is detected at pose `x_current` ← previously visited pose `x_old`,
add a constraint to the pose graph and reoptimise.

---

## 6. Graph SLAM

Graph SLAM decouples front-end (odometry/scan matching) from back-end
(global pose optimisation).

```
Front-end:
  pose graph nodes:   x₁, x₂, ..., xₜ
  odometry edges:     (xᵢ, xᵢ₊₁, Δx, Σ)
  loop closure edges: (xᵢ, xⱼ, Δx, Σ)

Back-end:
  minimise: Σ eᵢⱼᵀ · Ωᵢⱼ · eᵢⱼ    where eᵢⱼ = f(xᵢ, xⱼ) - Δxᵢⱼ
  solution: sparse linear system (Cholesky or Conjugate Gradient)
```

The back-end is a sparse nonlinear least squares problem. Libraries:
- **g2o** — used by Cartographer, RTAB-Map
- **GTSAM** — factor graph formulation
- **iSAM2** — incremental smoothing (best for online SLAM)

### Toy 1D pose graph

```cpp
struct Edge { int from, to; double delta, information; };

// Minimise Σ info_ij * (x[j] - x[i] - delta_ij)²
// Gradient descent step:
for (const auto& e : edges) {
    const double err = states[e.to] - states[e.from] - e.delta;
    const double grad = 2.0 * e.information * err;
    states[e.from] += lr * grad;
    states[e.to]   -= lr * grad;
}
// Fix gauge: pin states[0] = 0
```

---

## 7. Production SLAM Systems

| System | Backend | Front-end | Use case |
|---|---|---|---|
| Cartographer (Google) | Graph SLAM (g2o) | 2D/3D LiDAR | Warehouse, outdoor |
| RTAB-Map | Graph SLAM | LiDAR + camera | Indoor, loop closure |
| GMapping | Particle filter | 2D LiDAR | Simple indoor |
| KISS-ICP | ICP + point cloud | 3D LiDAR | Fast odometry |
| LIO-SAM | Factor graph (GTSAM) | LiDAR + IMU | Outdoor 3D |

### Cartographer in ROS2

```bash
ros2 launch cartographer_ros cartographer.launch.py \
  use_sim_time:=true \
  configuration_directory:=/path/to/config \
  configuration_basename:=my_robot.lua
```

Key configuration parameters:
```lua
-- my_robot.lua
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
POSE_GRAPH.optimize_every_n_nodes = 90
```

---

## 8. SLAM vs Localization vs Odometry

| Capability | Odometry | Localization (AMCL) | SLAM (Cartographer) |
|---|---|---|---|
| Requires map | No | Yes | No |
| Drift | Grows unboundedly | Bounded by map | Bounded by loop closures |
| Global consistency | No | Yes | Yes (after closure) |
| Map output | No | No | Yes |
| Compute | Low | Moderate | High |
| When to use | Short corridors | Known environment | Exploration / new space |

---

## 9. Occupancy Grid Builder — C++ Example

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

---

## 10. Interview Questions

**Q: What is the curse of dimensionality in EKF SLAM?**
A: The state vector grows as 3+2N (N landmarks); the covariance matrix grows
as (3+2N)². For 100 landmarks: covariance is 203×203 = 41K elements.
For 1000 landmarks: 2003×2003 ≈ 4M elements. Update cost grows as O(N²).
Graph SLAM avoids this by keeping a sparse factor graph.

**Q: Why does loop closure matter?**
A: Odometry drift accumulates over long paths. Without loop closure, the map
is inconsistent — the robot thinks it's at a different position when it returns
to the start. Loop closure constraints the full trajectory and map simultaneously.

**Q: What is the difference between online SLAM and full SLAM?**
A: Online SLAM maintains p(**x**t, m | **z**₁:t) — only the current pose.
Full SLAM maintains p(**x₁:t**, m | **z₁:t**) — the full trajectory. Graph
SLAM is a form of full SLAM; EKF SLAM is online.

**Q: How does Cartographer handle loop closure?**
A: It maintains a pose graph where nodes are submaps and edges are scan matches.
When a new scan matches a previously built submap above a threshold, a loop
closure edge is added. The back-end (Ceres) then globally optimises the graph.

**Q: Why use log-odds for occupancy grids?**
A: Log-odds transforms probabilities to an unconstrained real line: l ∈ (-∞, ∞).
Updates are simply additive: l_new = l_old + sensor_model_log_odds. Converting
back: p = 1/(1+exp(-l)). Numerically stable and computationally cheap.
