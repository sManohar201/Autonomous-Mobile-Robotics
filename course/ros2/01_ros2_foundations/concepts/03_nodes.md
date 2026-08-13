# Nodes

A node is a unit of responsibility. Good production nodes have narrow ownership:

- One node owns a sensor driver.
- One node owns a filter.
- One node owns a planner.
- One node owns robot health aggregation.

Avoid "god nodes" that subscribe to everything and decide everything. They become difficult to test, restart, and reason about.
