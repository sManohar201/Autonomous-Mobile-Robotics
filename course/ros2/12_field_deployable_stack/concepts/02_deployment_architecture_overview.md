# 1. Deployment Architecture Overview

A field-deployable ROS2 stack has distinct layers:

```
┌─────────────────────────────────────────────────────────┐
│  Operator interface                                       │
│    /deployment/readiness   /operator/health              │
│    /operator/fault_report  /robot/degraded_mode          │
├─────────────────────────────────────────────────────────┤
│  Stack orchestration                                      │
│    deployment_checklist_node  degraded_mode_manager      │
│    diagnostic_aggregator      fault_report_hard          │
├─────────────────────────────────────────────────────────┤
│  Subsystems                                               │
│    localization  planning  control  sensor_health        │
│    each publishes → /subsystem/status                    │
├─────────────────────────────────────────────────────────┤
│  Drivers / Simulation                                     │
│    hardware drivers  OR  ros_gz_bridge + Gazebo          │
├─────────────────────────────────────────────────────────┤
│  Configuration                                            │
│    config/robots/<name>.yaml  config/envs/<env>.yaml    │
└─────────────────────────────────────────────────────────┘
```
