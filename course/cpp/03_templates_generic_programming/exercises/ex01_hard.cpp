// Exercise 01 (Hard) - Template Deduction, Forwarding, and Overload Traps
//
// Context:
//   Robotics utility libraries often expose templated helpers for message
//   forwarding. A wrong reference category silently adds copies or binds to
//   the wrong overload.
//
// Pre-coding questions:
//   Q1. What is the difference between `T&&` in a deduced template and
//       `std::vector<T>&&`?
//   Q2. Why does plain `auto` drop references and top-level const?
//   Q3. What does `std::forward<T>(x)` preserve that `std::move(x)` does not?
//
// Tasks:
//   1. Implement category_name(T&&) returning "lvalue" or "rvalue".
//   2. Implement forward_to_sink(T&&, Sink&) preserving value category.
//   3. Add assertions proving lvalues and rvalues call different overloads.

#include <iostream>

int main() {
    std::cout << "ex01_hard passed\n";
}

