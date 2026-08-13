# 10. Interview Questions

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
