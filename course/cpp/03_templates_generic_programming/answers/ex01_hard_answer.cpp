#include <cassert>
#include <iostream>
#include <string>
#include <type_traits>
#include <utility>

template <typename T>
std::string category_name(T&&) {
    if constexpr (std::is_lvalue_reference_v<T>) return "lvalue";
    return "rvalue";
}

struct Sink {
    int lvalues{0};
    int rvalues{0};

    void accept(const int&) { ++lvalues; }
    void accept(int&&) { ++rvalues; }
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
    forward_to_sink(x, sink);
    forward_to_sink(4, sink);
    assert(sink.lvalues == 1);
    assert(sink.rvalues == 1);
    std::cout << "ex01_hard_answer passed\n";
}

