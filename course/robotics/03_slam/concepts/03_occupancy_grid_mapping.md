# 2. Occupancy Grid Mapping

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
