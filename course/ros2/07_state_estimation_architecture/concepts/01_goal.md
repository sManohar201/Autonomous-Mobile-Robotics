# Goal

Design a localization subsystem with clean ROS2 interfaces: measurement adapters
that decouple filter logic from message types, rejection logic (wrong frame, high
covariance, outlier jumps), a reset service, and diagnostics. The filter core
should be testable in isolation without a running ROS2 system.

> **See also:** https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html
> for executor patterns used to spin the state estimation node. Module 06 covers
> sensor topic QoS that feeds this subsystem.
