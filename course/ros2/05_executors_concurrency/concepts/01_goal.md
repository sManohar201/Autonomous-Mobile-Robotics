# Goal

Understand how ROS2 dispatches callbacks, when callbacks can overlap, what
"thread-safe" means in a ROS2 node, and how to protect shared sensor caches
without either deadlocking or busy-waiting.

> **See also:** https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html
> The official page covers the executor lifecycle and the distinction between
> spinning strategies. Come back here for the production trade-offs that page
> doesn't address.
