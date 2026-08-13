# 6. Trajectory Optimisation

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
