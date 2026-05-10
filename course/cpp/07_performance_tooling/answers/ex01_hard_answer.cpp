#include <algorithm>
#include <cassert>
#include <chrono>
#include <iostream>
#include <vector>

template <typename Fn>
double time_once_ms(Fn&& fn) {
    auto start = std::chrono::steady_clock::now();
    fn();
    auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
}

template <typename Fn>
double median_time_ms(int runs, Fn&& fn) {
    std::vector<double> times;
    times.reserve(static_cast<std::size_t>(runs));
    for (int i = 0; i < runs; ++i) times.push_back(time_once_ms(fn));
    std::sort(times.begin(), times.end());
    return times[times.size() / 2];
}

int main() {
    volatile int sink = 0;
    double t = median_time_ms(3, [&] { for (int i = 0; i < 1000; ++i) sink += i; });
    assert(t >= 0.0);
    std::cout << "ex01_hard_answer passed\n";
}

