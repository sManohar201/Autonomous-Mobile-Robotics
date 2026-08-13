# 5. Loop Closure

Loop closure is the moment the robot recognises a previously visited place.
It dramatically reduces accumulated drift.

```
Without loop closure:         With loop closure:
                              
  ╭──────────╮                ╭──────────╮
 path drift  ╯               path corrected ─── closure
  ←───────                                   ↗
  start                       start ─────────
```

### Detecting loop closure

**Scan-based:** Compare current LiDAR scan to stored scans using ICP. If ICP
converges with low RMS, close the loop.

**Feature-based:** Match visual (ORB, BRIEF) or LiDAR descriptors to a
database of visited poses.

### Closing the loop

Once a loop is detected at pose `x_current` ← previously visited pose `x_old`,
add a constraint to the pose graph and reoptimise.
