#include <chrono>
#include <iostream>

// Exercise: implement time_ms(callable) returning elapsed time in milliseconds.
//
// Use std::chrono::steady_clock (monotonic — safe for elapsed time).
// The function should be a template accepting any zero-argument callable.
// Use std::milli as the duration period for the return type.
//
// Hint: duration_cast or duration<double, ratio> both work — pick the simpler one.

int main() {
    // TODO
    std::cout << "ex01_benchmark_timer passed\n";
}
