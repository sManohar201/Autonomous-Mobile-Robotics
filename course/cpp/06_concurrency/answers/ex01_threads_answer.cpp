#include <cassert>
#include <iostream>
#include <thread>
#include <vector>

int main() {
    constexpr int n = 8;
    std::vector<int> values(n, -1);
    std::vector<std::thread> threads;
    threads.reserve(n);

    for (int i = 0; i < n; ++i) {
        threads.emplace_back([i, &values] { values[i] = i; });
    }
    for (auto& t : threads) t.join();

    for (int i = 0; i < n; ++i) assert(values[i] == i);
    std::cout << "ex01_threads_answer passed\n";
}

