# 6. Common Gazebo Bridge Issues

| Symptom | Cause | Fix |
|---|---|---|
| No messages on `/imu/data` | Bridge not running or topic name mismatch | `ros2 topic list` — check both sides |
| Timestamps at 0 | `use_sim_time` mismatch or clock not started | Check `ros2 param get /node use_sim_time` |
| Wrong `frame_id` | Bridge remapping doesn't match URDF frame | Match bridge config to URDF sensor frames |
| RViz model blinks | Multiple `/clock` publishers; non-monotonic tf | Check `ros2 topic info /clock --verbose`, kill stale bridges |
| "Moved backwards in time" | Node started before `/clock` | Use `TimerAction(5s)` delay in launch file |

### Debugging tools

```bash
# Check clock source and rate
ros2 topic hz /clock           # should be ~1000 Hz in Gazebo
ros2 topic info /clock --verbose  # shows all publishers

# Check what the bridge is exposing
ros2 topic list | grep -v /rosout | sort

# Check frame IDs on sensor topics
ros2 topic echo /imu/data --field header.frame_id --once
ros2 topic echo /front_laser/scan --field header.frame_id --once

# Check QoS compatibility
ros2 topic info /front_laser/scan --verbose
```
