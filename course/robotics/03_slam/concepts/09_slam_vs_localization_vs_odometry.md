# 8. SLAM vs Localization vs Odometry

| Capability | Odometry | Localization (AMCL) | SLAM (Cartographer) |
|---|---|---|---|
| Requires map | No | Yes | No |
| Drift | Grows unboundedly | Bounded by map | Bounded by loop closures |
| Global consistency | No | Yes | Yes (after closure) |
| Map output | No | No | Yes |
| Compute | Low | Moderate | High |
| When to use | Short corridors | Known environment | Exploration / new space |
