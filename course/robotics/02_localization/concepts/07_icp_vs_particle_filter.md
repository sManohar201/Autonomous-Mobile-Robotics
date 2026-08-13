# 6. ICP vs Particle Filter

| Property | ICP | Particle Filter |
|---|---|---|
| Problem solved | Scan matching (local) | Global + tracking |
| Initial guess needed | Yes (within ~0.5 m) | No (global localization) |
| Map type | Point cloud | Occupancy grid |
| Computational cost | O(N log N) per iteration | O(N particles · beams) |
| Output | Transform (R, t) | Probability distribution |
| Handles multimodal belief | No | Yes |
| Used in | LiDAR SLAM, ICP-based localization | AMCL, MCL |

In practice: AMCL (particle filter) for 2D navigation, ICP or NDT for 3D SLAM.
