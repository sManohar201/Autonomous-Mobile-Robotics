# 5. Potential Fields

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
