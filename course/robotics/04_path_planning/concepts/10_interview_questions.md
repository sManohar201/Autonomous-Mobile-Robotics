# 9. Interview Questions

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
