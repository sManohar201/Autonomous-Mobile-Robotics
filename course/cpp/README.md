# C++ for Robotics Software Engineering

Eight modules from foundations to production robotics patterns. No ROS2 dependency — all exercises compile with plain `g++` and `cmake`.

Each module ends with a **capstone project** that requires integrating everything covered so far. Exercises and projects are designed to require real thinking — multiple valid approaches exist, tradeoffs must be reasoned about, and the math is real.

## Modules

| # | Module | Key concepts | Capstone project |
|---|--------|-------------|-----------------|
| 01 | [Foundations & Toolchain](01_foundations/concepts.md) | Compilation pipeline, types, const, refs, pointers, namespaces, CMake | `Vector2d` math library |
| 02 | [Classes, OOP & RAII](02_classes_oop_raii/concepts.md) | Classes, invariants, copy/move semantics, RAII, smart pointers, inheritance, virtual dispatch, operator overloading, factory pattern | 1D Kalman filter integrating all patterns |
| 03 | [Templates & Generic Programming](03_templates_generic_programming/concepts.md) | Function/class templates, partial specialisation, CRTP, type traits, `if constexpr`, C++20 concepts | Generic ring buffer + type-safe physical units |
| 04 | [Modern C++](04_modern_cpp/concepts.md) | `auto`, structured bindings, lambdas, `std::function`, `optional`, `variant`, `constexpr if`, fold expressions | Config parser + compile-time state machine |
| 05 | [Standard Library](05_standard_library/concepts.md) | Containers, iterators, algorithms, `chrono`, `string_view`, custom allocators | Sensor data pipeline with time-based windowing |
| 06 | [Concurrency](06_concurrency/concepts.md) | Threads, mutex, `condition_variable`, `atomic`, futures, thread pool | Bounded blocking queue with producer/consumer |
| 07 | [Performance & Tooling](07_performance_tooling/concepts.md) | Cache hierarchy, SIMD basics, sanitisers, GDB, `perf`, CMake advanced, benchmarking | Cache-aware point cloud transform |
| 08 | [Robotics Patterns](08_robotics_patterns/concepts.md) | Eigen, observer pattern, plugin architecture, FSM, component entity system | Mini localisation library |

## Difficulty

Exercises are designed at **graduate course level** — the kind of problems set in Stanford CS107, MIT 6.172, or CMU 15-410. Each exercise:

- Has a clear problem statement but **no obvious single solution** — design decisions are yours to make
- Contains **intentional traps** that look correct but produce subtle bugs (non-commutativity, double-free, MIL order, circular index off-by-one)
- Requires **mathematical reasoning** alongside C++ (positive-definiteness, SE(2) group theory, Bayesian fusion, Kalman math)
- Often has **design questions** that must be answered before coding begins

If an exercise takes less than 2 hours, you probably skipped thinking about the tradeoffs.

## Progression

Each module builds on the last. The Module 8 capstone is a small but complete localisation library that uses every pattern from every module — the same architecture as `robot_localization` at its core.
