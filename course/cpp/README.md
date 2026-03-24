# C++ for Robotics Software Engineering

Nine modules from foundations to production robotics patterns. No ROS2 dependency — all exercises compile with plain `g++` and `cmake`.

## Modules

| # | Module | Key concepts | Project |
|---|--------|-------------|---------|
| 01 | [Foundations & Toolchain](01_foundations/concepts.md) | Compilation, types, const, refs, pointers, namespaces, CMake | 2D vector math library |
| 02 | Memory & Ownership | Stack/heap, RAII, unique_ptr, shared_ptr, move semantics | Sensor reading buffer |
| 03 | OOP & Polymorphism | Classes, virtual dispatch, abstract base, design patterns | Filter hierarchy (Base→EKF/UKF stub) |
| 04 | Templates | Function/class templates, CRTP, type traits | Generic ring buffer |
| 05 | Modern C++ | auto, lambdas, optional, variant, constexpr | Config parser |
| 06 | Standard Library | Containers, algorithms, chrono | Sensor data pipeline |
| 07 | Concurrency | Threads, mutex, condition_variable, atomic | Thread-safe scan queue |
| 08 | Performance & Tooling | Optimisation, sanitisers, GDB, CMake advanced | Benchmark suite |
| 09 | Robotics Patterns | Eigen, callbacks, plugin arch, FSM | Mini localisation library |

## Progression

Each module builds on the last. The project at the end of Module 9 is a small but complete localisation library that uses every pattern from every module.
