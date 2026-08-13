# 7. Nav2 Planner Stack

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
