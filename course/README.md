# Robotics Software Engineering Course

A from-scratch, open-source course for mastering **C++** and **ROS2** through the lens of robotics software engineering. The goal is not just to use robotics frameworks — it is to understand them deeply enough to design and build them.

---

## Who this is for

- Software engineers moving into robotics
- Robotics engineers who want to go deeper on C++ and ROS2 internals
- Anyone preparing for senior robotics software engineering interviews

No prior robotics knowledge is assumed. C++ is taught from foundations up through advanced patterns. ROS2 builds directly on top of the C++ module.

---

## Structure

```
course/
├── cpp/        ← C++ module (no ROS2 dependency)
└── ros2/       ← ROS2 module (builds on C++ module)
```

Each module contains numbered units. Every unit follows the same format:

| File | Purpose |
|------|---------|
| `concepts/` | One markdown file per topic (theory + code examples) |
| `exercises/` | Standalone `.cpp` files — compile and run locally |
| `answers/` | Complete implementations with explanations (do not read before attempting) |
| `quiz/` | One markdown file per quiz question, with hidden answers |

Complete all exercises, pass the quiz, and finish the capstone project before moving to the next module.

---

## C++ Modules

| # | Module | Topics |
|---|--------|--------|
| 01 | [Foundations & Toolchain](cpp/01_foundations/concepts/) | Compilation pipeline, types, const, references, pointers, namespaces, CMake |
| 02 | [Classes, OOP & RAII](cpp/02_classes_oop_raii/concepts/) | Classes, invariants, copy/move semantics, Rule of Five, RAII, smart pointers, inheritance, virtual dispatch, operator overloading, factory pattern |
| 03 | [Templates & Generic Programming](cpp/03_templates_generic_programming/concepts/) | Function/class templates, partial specialisation, CRTP, type traits, `if constexpr`, C++20 concepts |
| 04 | [Modern C++](cpp/04_modern_cpp/concepts/) | `auto`, structured bindings, lambdas, `std::function`, `optional`, `variant`, `constexpr if` |
| 05 | [Standard Library](cpp/05_standard_library/concepts/) | Containers, iterators, algorithms, `chrono`, `string_view` |
| 06 | [Concurrency](cpp/06_concurrency/concepts/) | Threads, mutex, `condition_variable`, `atomic`, futures, thread pool |
| 07 | [Performance & Tooling](cpp/07_performance_tooling/concepts/) | Cache hierarchy, SIMD basics, sanitisers, GDB, `perf`, CMake advanced |
| 08 | [Robotics Patterns](cpp/08_robotics_patterns/concepts/) | Eigen, observer pattern, plugin architecture, FSM, component entity system |

---

## Course Composition

Each C++ module is composed of:

- Concept notes with robotics-oriented examples
- Moderate standalone exercises for the core topic
- Hard exercises with design questions, debugging scenarios, and implementation tasks
- Answer files with complete reference implementations
- A quiz for reviewing terminology, reasoning, and code behavior
- A capstone-style exercise that combines the module's main ideas

---

## How to use

1. Read the topic files in `concepts/` — skim what you know, read carefully what you don't
2. Answer the design questions in each exercise file before writing any code
3. Compile and test (`cmake -B build && cmake --build build`)
4. Only check `answers/` after you have a working solution or are genuinely stuck
5. Pass the quiz
6. Build the end-of-module capstone project

---

## Building exercises

Each exercise module has a `CMakeLists.txt`. Build with:

```bash
cd course/cpp/01_foundations/exercises
cmake -B build && cmake --build build
./build/<exercise_name>
```

Requirements: `g++` with C++17 support, `cmake` ≥ 3.16.

---

## ROS2 Module

The [ROS2 module](ros2/README.md) starts with software architecture fundamentals and introduces Gazebo gradually as an integration environment. It covers nodes, interfaces, package boundaries, lifecycle nodes, QoS, executors, diagnostics, testing, launch, sensor integration, simulation, and field deployment patterns.

## Interview Prep

For live robotics assignment preparation, see [C++, ROS2 Architecture, Nav2, Sensor Fusion, and PCL Interview Prep](interview_prep/nav2_sensor_fusion_pcl/README.md). It turns resume-level robotics experience into C++ design fluency, ROS2 architecture explanations, trade-off language, debugging flows, and assignment drills.
