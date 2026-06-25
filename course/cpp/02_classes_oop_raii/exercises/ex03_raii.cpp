// Exercise 03 — RAII
//
// EXPECTED OUTPUT:
//   [timer] ScopedTimer started
//   processing...
//   [timer] ScopedTimer stopped: ~0 ms
//   [lock] acquired
//   critical section
//   [lock] released
//   [lock] acquired
//   critical section (will throw)
//   [lock] released
//   caught: simulated error

#include <iostream>
#include <chrono>
#include <stdexcept>
#include <string>

class ScopedTimer {
public:
    explicit ScopedTimer(const std::string& label)
        : label_(label), start_(std::chrono::steady_clock::now())
    {
        std::cout << "[timer] " << label_ << " started\n";
    }

    ~ScopedTimer() {
        auto end = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_).count();
        std::cout << "[timer] " << label_ << " stopped: " << ms << " ms\n";
    }

private:
    std::string label_;
    std::chrono::steady_clock::time_point start_;
};


struct FakeMutex {
    void lock()   { std::cout << "[lock] acquired\n"; }
    void unlock() { std::cout << "[lock] released\n"; }
};

class ScopedLock {
public:
    explicit ScopedLock(FakeMutex& m) : mutex_(m) {
        mutex_.lock();
    }

    ~ScopedLock() {
        mutex_.unlock();
    }

    ScopedLock(const ScopedLock&)            = delete;
    ScopedLock& operator=(const ScopedLock&) = delete;

private:
    FakeMutex& mutex_;
};


void simulate_work() {
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(1);
    while (std::chrono::steady_clock::now() < end) {}
}

int main() {
    {
        ScopedTimer t("ScopedTimer");
        simulate_work();
        std::cout << "processing...\n";
    }

    FakeMutex mtx;
    {
        ScopedLock lk(mtx);
        std::cout << "critical section\n";
    }

    try {
        ScopedLock lk(mtx);
        std::cout << "critical section (will throw)\n";
        throw std::runtime_error("simulated error");
    } catch (const std::exception& e) {
        std::cout << "caught: " << e.what() << "\n";
    }

    return 0;
}
