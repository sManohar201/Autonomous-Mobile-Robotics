# 4.1 What "Modern C++" Means

Modern C++ is not a list of syntax tricks. It is a shift toward:

- Expressing intent directly
- Reducing manual type noise
- Avoiding invalid sentinel values
- Making ownership and absence explicit
- Moving decisions from runtime to compile time when practical

Robotics code benefits from this because failures often happen at subsystem boundaries: configuration files, sensor messages, state machines, and callback pipelines. Modern C++ gives you stronger vocabulary types for those boundaries.
