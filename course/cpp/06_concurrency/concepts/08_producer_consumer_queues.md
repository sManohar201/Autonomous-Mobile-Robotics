# 6.7 Producer/Consumer Queues

A common robotics pattern is one thread producing sensor messages and another consuming them.

The queue needs:

- A mutex to protect storage
- A condition variable to block consumers
- A shutdown flag
- A capacity policy for overload

For real-time systems, prefer bounded queues and explicit drop behavior.
