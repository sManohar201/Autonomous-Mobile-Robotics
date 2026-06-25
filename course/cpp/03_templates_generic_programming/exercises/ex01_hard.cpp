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
//   A1. In a deduced template `template<typename T> void f(T&&)`, `T&&` is a
//       forwarding reference (universal reference). Passing an lvalue deduces
//       T as T& and T&& collapses to T& — the function receives an lvalue.
//       Passing an rvalue deduces T as T and T&& stays T&& — rvalue path.
//       `std::vector<T>&&` is never a forwarding reference — T is not deduced
//       from that parameter, so it is always an rvalue reference.
//
//   Q2. Why does plain `auto` drop references and top-level const?
//   A2. `auto` follows template argument deduction rules: like `T` in
//       `template<typename T> f(T)`, it strips reference and top-level
//       cv-qualifiers. Use `auto&`, `auto&&`, or `decltype(auto)` to preserve.
//
//   Q3. What does `std::forward<T>(x)` preserve that `std::move(x)` does not?
//   A3. `std::move` always casts to rvalue. `std::forward<T>(x)` casts to
//       rvalue only when T is NOT a reference type (rvalue origin). If T is
//       T& (lvalue reference), forward returns an lvalue — preserving the
//       original value category. This is essential in generic code to avoid
//       stealing resources from lvalue arguments.
//
// Tasks:
//   1. Implement category_name(T&&) returning "lvalue" or "rvalue".
//   2. Implement forward_to_sink(T&&, Sink&) preserving value category.
//   3. Add assertions proving lvalues and rvalues call different overloads.

#include <cassert>
#include <iostream>
#include <string>
#include <type_traits>
#include <utility>

// When T&& is a forwarding reference: T deduces as T& for lvalue, T for rvalue.
template <typename T>
std::string category_name(T&&) {
    if constexpr (std::is_lvalue_reference_v<T>) return "lvalue";
    return "rvalue";
}

struct Sink {
    int lvalues{0};
    int rvalues{0};

    void accept(const int&) { ++lvalues; }
    void accept(int&&)      { ++rvalues; }
};

template <typename T>
void forward_to_sink(T&& value, Sink& sink) {
    sink.accept(std::forward<T>(value));
}

int main() {
    int x = 3;
    assert(category_name(x) == "lvalue");
    assert(category_name(3) == "rvalue");

    Sink sink;
    forward_to_sink(x, sink);   // lvalue — accept(const int&)
    forward_to_sink(4, sink);   // rvalue — accept(int&&)
    assert(sink.lvalues == 1);
    assert(sink.rvalues == 1);

    std::cout << "ex01_hard passed\n";
}

