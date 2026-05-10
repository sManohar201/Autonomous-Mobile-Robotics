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
| `concepts.md` | Full theory + code examples, beginner → advanced, with robotics context throughout |
| `exercises/` | Standalone `.cpp` files — compile and run locally |
| `answers/` | Complete implementations with explanations (do not read before attempting) |
| `quiz.md` | Mixed-format quiz with hidden answers |

Complete all exercises, pass the quiz, and finish the capstone project before moving to the next module.

---

## C++ Modules

| # | Module | Topics |
|---|--------|--------|
| 01 | [Foundations & Toolchain](cpp/01_foundations/concepts.md) | Compilation pipeline, types, const, references, pointers, namespaces, CMake |
| 02 | [Classes, OOP & RAII](cpp/02_classes_oop_raii/concepts.md) | Classes, invariants, copy/move semantics, Rule of Five, RAII, smart pointers, inheritance, virtual dispatch, operator overloading, factory pattern |
| 03 | [Templates & Generic Programming](cpp/03_templates_generic_programming/concepts.md) | Function/class templates, partial specialisation, CRTP, type traits, `if constexpr`, C++20 concepts |
| 04 | [Modern C++](cpp/04_modern_cpp/concepts.md) | `auto`, structured bindings, lambdas, `std::function`, `optional`, `variant`, `constexpr if` |
| 05 | [Standard Library](cpp/05_standard_library/concepts.md) | Containers, iterators, algorithms, `chrono`, `string_view` |
| 06 | [Concurrency](cpp/06_concurrency/concepts.md) | Threads, mutex, `condition_variable`, `atomic`, futures, thread pool |
| 07 | [Performance & Tooling](cpp/07_performance_tooling/concepts.md) | Cache hierarchy, SIMD basics, sanitisers, GDB, `perf`, CMake advanced |
| 08 | [Robotics Patterns](cpp/08_robotics_patterns/concepts.md) | Eigen, observer pattern, plugin architecture, FSM, component entity system |

---

## Difficulty

Exercises are written at **graduate course level** (Stanford, MIT, CMU). They are not fill-in-the-blank — they require:

- Making design decisions (no single correct structure is given)
- Mathematical reasoning alongside C++ (SE(2) group theory, Bayesian inference, positive-definiteness)
- Reasoning about subtle bugs that look correct at first glance
- Answering design questions in comments **before** coding

Budget 2–4 hours per exercise set. The capstone projects are full multi-day assignments.

---

## How to use

1. Read `concepts.md` — skim what you know, read carefully what you don't
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

Coming after the C++ module is complete. Covers nodes, pub/sub, services, TF2, lifecycle nodes, composable nodes, diagnostics, launch files, and bags. All examples run against the simulation robot in this repository.
