#include <atomic>
#include <cassert>
#include <future>
#include <iostream>

int sum_to(int n) {
    int sum = 0;
    for (int i = 1; i <= n; ++i) sum += i;
    return sum;
}

int main() {
    std::atomic<bool> stop{false};
    assert(!stop.load());
    stop.store(true);
    assert(stop.load());

    auto fut = std::async(std::launch::async, sum_to, 100);
    assert(fut.get() == 5050);
    std::cout << "ex04_atomic_future_answer passed\n";
}

