// Exercise 03 — RAII
// ANSWER FILE

#include <iostream>
#include <chrono>
#include <stdexcept>
#include <string>

// ── Part A: ScopedTimer ──────────────────────────────────────────────────────

class ScopedTimer {
public:
    explicit ScopedTimer(const std::string& label)
        : label_(label), start_(std::chrono::steady_clock::now())
    {
        std::cout << "[timer] " << label_ << " started\n";
    }

    ~ScopedTimer() {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_).count();
        std::cout << "[timer] " << label_ << " stopped: " << elapsed << " ms\n";
    }

private:
    std::string label_;
    std::chrono::steady_clock::time_point start_;
};


// ── Part B: ScopedLock ───────────────────────────────────────────────────────

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

    // Locks must not be duplicated — deleting copy semantics enforces unique ownership.
    ScopedLock(const ScopedLock&)            = delete;
    ScopedLock& operator=(const ScopedLock&) = delete;

private:
    FakeMutex& mutex_;
};


// ── main ─────────────────────────────────────────────────────────────────────

void simulate_work() {
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(1);
    while (std::chrono::steady_clock::now() < end) {}
}

int main() {
    // Part A
    {
        ScopedTimer t("ScopedTimer");
        simulate_work();
        std::cout << "processing...\n";
    }

    // Part B — normal path
    FakeMutex mtx;
    {
        ScopedLock lk(mtx);
        std::cout << "critical section\n";
    }

    // Part B — exception path (lock must still be released)
    try {
        ScopedLock lk(mtx);
        std::cout << "critical section (will throw)\n";
        throw std::runtime_error("simulated error");
    } catch (const std::exception& e) {
        std::cout << "caught: " << e.what() << "\n";
    }

    return 0;
}
