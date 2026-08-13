# Goal

Use Gazebo Harmonic as an integration environment while keeping simulation-specific
wiring completely separate from production node logic. A node that monitors sensor
health should run unchanged on hardware and in simulation — only the launch file
and bridge configuration differ.

> **See also:** https://gazebosim.org/docs/harmonic/ros2_integration
> for the official ros_gz_bridge setup guide. This module focuses on the
> architectural discipline needed to keep simulation wiring out of business logic.
