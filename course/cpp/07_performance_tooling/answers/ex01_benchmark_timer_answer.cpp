#include <cassert>
#include <chrono>
#include <iostream>

template <typename Callable>
double time_ms(Callable&& callable) {
    const auto start = std::chrono::steady_clock::now();
    callable();
    const auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
}

int main() {
    volatile int sink = 0;
    const double elapsed = time_ms([&] {
        for (int i = 0; i < 1000; ++i) sink += i;
    });
    assert(elapsed >= 0.0);
    std::cout << "ex01_benchmark_timer_answer passed\n";
}

