# Goal

Subscribe to real or simulated robot sensor streams correctly: verify frame IDs,
check timestamps, monitor publication rates, detect stale inputs, and publish a
health verdict. These patterns appear in every production sensor fusion pipeline.

> **See also:** https://docs.ros.org/en/jazzy/Concepts/Basic/About-ROS-Interfaces.html
> for the message type catalogue. Sensor-specific message definitions live in
> `sensor_msgs`, `nav_msgs`, and `geometry_msgs`.
