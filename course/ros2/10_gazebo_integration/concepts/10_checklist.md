# 9. Checklist

- [ ] `use_sim_time: true` set for all nodes in simulation launch file
- [ ] Bridge started first — before nodes that consume sensor topics
- [ ] Nodes don't subscribe to Gazebo transport topic names directly
- [ ] `frame_id` in sensor messages matches URDF sensor frames
- [ ] Startup delay handles clock-before-nodes ordering
- [ ] Monitoring node reports `WAITING_FOR_GAZEBO_CLOCK` during startup grace period
- [ ] All topic names that differ between sim and hardware are launch arguments, not hardcoded
