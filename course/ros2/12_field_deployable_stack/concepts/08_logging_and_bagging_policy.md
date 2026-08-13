# 7. Logging and Bagging Policy

### Pre-mission

```bash
# Start recording key topics before mission
ros2 bag record \
  /odom /imu/data /front_laser/scan \
  /localization/status /subsystem/status /operator/health \
  /robot/degraded_mode /deployment/readiness \
  -o mission_$(date +%Y%m%d_%H%M%S)
```

### Incident-triggered recording

```python
# In a monitoring node: start bag on fault
if mode == "STOP" and not bag_running:
    subprocess.Popen(['ros2', 'bag', 'record', '-a', '-o', 'incident_bag'])
    bag_running = True
```

### Retention policy

| Data | Retention | Reason |
|---|---|---|
| Normal mission bags | 7 days | Storage cost vs. debugging value |
| Incident bags | 90 days | Required for post-mortem |
| CI regression bags | Indefinite | Test fixtures — never delete |
