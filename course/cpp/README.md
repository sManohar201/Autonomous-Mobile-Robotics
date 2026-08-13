# C++ for Robotics Software Engineering

Eight modules from foundations to production robotics patterns. No ROS2 dependency — all exercises compile with plain `g++` and `cmake`.

Each module includes concept notes, moderate exercises, hard exercises, answer implementations, a quiz, and a capstone-style project that integrates the module's main ideas.

## Modules

| # | Module | Key concepts | Capstone project |
|---|--------|-------------|-----------------|
| 01 | [Foundations & Toolchain](01_foundations/concepts/) | Compilation pipeline, types, const, refs, pointers, namespaces, CMake | `Vector2d` math library |
| 02 | [Classes, OOP & RAII](02_classes_oop_raii/concepts/) | Classes, invariants, copy/move semantics, RAII, smart pointers, inheritance, virtual dispatch, operator overloading, factory pattern | 1D Kalman filter integrating all patterns |
| 03 | [Templates & Generic Programming](03_templates_generic_programming/concepts/) | Function/class templates, partial specialisation, CRTP, type traits, `if constexpr`, C++20 concepts | Generic ring buffer + type-safe physical units |
| 04 | [Modern C++](04_modern_cpp/concepts/) | `auto`, structured bindings, lambdas, `std::function`, `optional`, `variant`, `constexpr if`, fold expressions | Config parser + compile-time state machine |
| 05 | [Standard Library](05_standard_library/concepts/) | Containers, iterators, algorithms, `chrono`, `string_view`, custom allocators | Sensor data pipeline with time-based windowing |
| 06 | [Concurrency](06_concurrency/concepts/) | Threads, mutex, `condition_variable`, `atomic`, futures, thread pool | Bounded blocking queue with producer/consumer |
| 07 | [Performance & Tooling](07_performance_tooling/concepts/) | Cache hierarchy, SIMD basics, sanitisers, GDB, `perf`, CMake advanced, benchmarking | Cache-aware point cloud transform |
| 08 | [Robotics Patterns](08_robotics_patterns/concepts/) | Eigen, observer pattern, plugin architecture, FSM, component entity system | Mini localisation library |

## Course Composition

Each module contains:

- `concepts/` — one markdown file per topic, plus `README.md` listing them in order
- `exercises/` with moderate standalone tasks
- `exercises/*_hard.cpp` with extended design and implementation tasks
- `answers/` with complete reference implementations
- `quiz/` — one markdown file per review question, plus `README.md` listing them in order
- A capstone-style exercise that combines the module's main concepts

## Progression

Each module builds on the last. The Module 8 capstone is a small but complete localisation library that uses every pattern from every module — the same architecture as `robot_localization` at its core.
