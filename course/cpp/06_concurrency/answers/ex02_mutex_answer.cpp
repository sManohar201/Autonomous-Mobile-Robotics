#include <cassert>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

class ThreadSafeCounter {
public:
    void increment() {
        std::lock_guard<std::mutex> lock{mutex_};
        ++value_;
    }

    int value() const {
        std::lock_guard<std::mutex> lock{mutex_};
        return value_;
    }

private:
    mutable std::mutex mutex_;
    int value_{0};
};

int main() {
    ThreadSafeCounter counter;
    std::vector<std::thread> threads;
    for (int t = 0; t < 4; ++t) {
        threads.emplace_back([&] {
            for (int i = 0; i < 1000; ++i) counter.increment();
        });
    }
    for (auto& thread : threads) thread.join();
    assert(counter.value() == 4000);
    std::cout << "ex02_mutex_answer passed\n";
}

