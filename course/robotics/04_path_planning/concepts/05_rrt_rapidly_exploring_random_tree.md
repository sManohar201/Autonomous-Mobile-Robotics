# 4. RRT (Rapidly-exploring Random Tree)

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
