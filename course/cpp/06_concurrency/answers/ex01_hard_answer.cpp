#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>

class Worker {
public:
    Worker() : thread_{[this] { run(); }} {}
    ~Worker() {
        stop_.store(true);
        if (thread_.joinable()) thread_.join();
    }
    int ticks() const { return ticks_.load(); }
private:
    void run() {
        while (!stop_.load()) {
            ++ticks_;
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }
    }
    std::atomic<bool> stop_{false};
    std::atomic<int> ticks_{0};
    std::thread thread_;
};

int main() {
    { Worker w; std::this_thread::sleep_for(std::chrono::milliseconds{5}); assert(w.ticks() > 0); }
    std::cout << "ex01_hard_answer passed\n";
}

