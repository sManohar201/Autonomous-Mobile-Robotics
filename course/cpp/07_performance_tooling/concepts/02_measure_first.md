# 7.1 Measure First

Performance work without measurement is guessing. Start with a clear workload, a baseline, and repeatable timing.

Use `std::chrono::steady_clock` for simple local measurements. Use profilers such as `perf`, `callgrind`, or tracing tools for real systems.
