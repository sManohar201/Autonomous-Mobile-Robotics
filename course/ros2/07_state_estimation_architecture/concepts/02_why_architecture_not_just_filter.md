# 1. Why "Architecture" Not Just "Filter"

A deployable localization system has layers:

```
                     ROS2 layer
   ┌────────────────────────────────────────────────────┐
   │  /odom subscription → MeasurementAdapter            │
   │  /localization/reset service                        │
   │  /localization/status publisher (diagnostics timer) │
   └──────────────────────────┬─────────────────────────┘
                              │ typed measurement struct
                     Filter layer (pure C++)
   ┌──────────────────────────────────────────────────────┐
   │  StateEstimator                                       │
   │    predict(stamp)                                     │
   │    update(measurement, &reason) → accepted/rejected   │
   │    reset()                                            │
   │    diagnostics(clock) → string                        │
   └──────────────────────────────────────────────────────┘
```

The separation exists because:
- **Testability:** The filter can be unit-tested with deterministic inputs, no
  DDS stack needed.
- **Portability:** The filter core can run in a simulation harness, on hardware,
  or in offline analysis tools.
- **Clarity:** Every ROS2 integration concern (frame IDs, timestamps, QoS)
  lives in the adapter; the filter sees clean structs.
