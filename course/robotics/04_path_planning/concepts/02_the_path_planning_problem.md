# 1. The Path Planning Problem

Given:
- Configuration space **C** — all possible robot states (positions, orientations)
- Obstacle region **C_obs** ⊂ **C** — states that cause collision
- Free space **C_free** = **C** \ **C_obs**
- Start **q_start** ∈ **C_free**
- Goal **q_goal** ∈ **C_free**

Find: a path τ: [0,1] → **C_free** with τ(0) = **q_start**, τ(1) = **q_goal**

For a circular robot, **C** = ℝ² and inflation of the map obstacles by the robot
radius reduces the problem to a point robot in free space.
