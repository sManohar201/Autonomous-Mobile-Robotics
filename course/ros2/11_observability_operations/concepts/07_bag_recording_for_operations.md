# 6. Bag Recording for Operations

A bag captures the full topic graph at a point in time. It is the primary
evidence preservation tool for post-mortem analysis.

```bash
# Record all topics to named bag
ros2 bag record -a -o fault_2026_06_25

# Record only the topics needed for localization post-mortem
ros2 bag record /odom /imu/data /front_laser/scan /localization/status \
  -o localization_incident

# Replay
ros2 bag play localization_incident

# Inspect
ros2 bag info localization_incident
```

### Operational bag strategy

| Use case | Approach |
|---|---|
| Continuous ring buffer | `--storage-preset sqlite3` + auto-split by size |
| Incident capture | Trigger bag record on fault event; stop after 60 s |
| Test fixtures | Record known-good runs; replay in CI for regression testing |
