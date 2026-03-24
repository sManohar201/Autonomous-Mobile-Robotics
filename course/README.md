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
| `concepts.md` | Full theory + code examples, beginner → advanced |
| `exercises/` | Standalone `.cpp` files — compile and run locally |
| `answers/` | Complete implementations with explanations |
| `quiz.md` | Mixed-format quiz with hidden answers |

Complete all exercises and pass the quiz before moving to the next unit. Each unit ends with a **project** that uses only concepts covered so far.

---

## C++ Modules

| Module | Topics |
|--------|--------|
| [01 — Foundations & Toolchain](cpp/01_foundations/concepts.md) | Compilation pipeline, types, const, references, pointers, namespaces, CMake |
| 02 — Memory & Ownership | Stack/heap, RAII, smart pointers, move semantics, Rule of 0/3/5 |
| 03 — OOP & Polymorphism | Classes, inheritance, virtual dispatch, abstract bases, design patterns |
| 04 — Templates | Function/class templates, specialisation, CRTP, type traits |
| 05 — Modern C++ | auto, lambdas, std::function, optional, variant, constexpr |
| 06 — Standard Library | Containers, iterators, algorithms, chrono, string |
| 07 — Concurrency | Threads, mutex, condition_variable, atomic, futures |
| 08 — Performance & Tooling | Optimisation, cache, sanitisers, GDB, CMake advanced |
| 09 — Robotics Patterns | Eigen, callbacks, observer pattern, plugin arch, FSMs |

---

## How to use

1. Read `concepts.md` — skim what you know, read carefully what you don't
2. Complete all exercises in order (`cmake -B build && cmake --build build`)
3. Check your work against the `answers/` directory
4. Pass the quiz
5. Build the end-of-module project

---

## Building exercises

Each exercise module has a `CMakeLists.txt`. Build with:

```bash
cd course/cpp/01_foundations/exercises
cmake -B build && cmake --build build
./build/<exercise_name>
```

Requirements: `g++` with C++17 support, `cmake` ≥ 3.15.
