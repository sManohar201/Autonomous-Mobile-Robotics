// Exercise 03 — RAII
//
// TASK:
//   Implement two RAII wrappers below.
//   Do not change main().
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

// ── Part A: ScopedTimer ──────────────────────────────────────────────────────
// Records the time at construction.
// On destruction, prints elapsed milliseconds.
//
class ScopedTimer {
public:
    // TODO: constructor — store label, record start time,
    //       print "[timer] <label> started\n"
    explicit ScopedTimer(const std::string& label);

    // TODO: destructor — compute elapsed ms,
    //       print "[timer] <label> stopped: <ms> ms\n"
    ~ScopedTimer();

private:
    std::string label_;
    std::chrono::steady_clock::time_point start_;
};


// ── Part B: ScopedLock ───────────────────────────────────────────────────────
// FakeMutex is provided — do not modify it.
struct FakeMutex {
    void lock()   { std::cout << "[lock] acquired\n"; }
    void unlock() { std::cout << "[lock] released\n"; }
};

class ScopedLock {
public:
    // TODO: constructor — call mutex.lock()
    explicit ScopedLock(FakeMutex& m);

    // TODO: destructor — call mutex_.unlock()
    ~ScopedLock();

    // TODO: delete copy constructor and copy assignment
    //       (why? — locks must not be duplicated)

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
