# 8. Path Planning in Simulation (This Repo)

```bash
# Launch simulation with Nav2
ros2 launch robot_description gazebo.launch.py world:=slam_district

# Launch Nav2 (in a second terminal)
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True

# Send a goal from RViz2: click "2D Nav Goal"
# Or from CLI:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {pose: {position: {x: 3.0, y: 2.0}}}}'
```

Monitor the planner:
```bash
ros2 topic echo /plan --field poses[0].pose.position
ros2 topic hz /cmd_vel
```
