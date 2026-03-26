// Exercise 03 — RAII with commit/rollback semantics: ScopedStateGuard
// ANSWER FILE
//
// ── Research Questions ───────────────────────────────────────────────────────
//
// A1 (rollback() then destructor):
//   rollback() restores state_ = saved_ immediately, then sets committed_ = true.
//   When the destructor runs, it checks !committed_ — which is false — so it
//   does nothing.  The state was restored exactly once, by rollback().
//
// A2 (commit() then destructor):
//   commit() sets committed_ = true.
//   When the destructor runs, it checks !committed_ — which is false — so it
//   does nothing.  The modified state is permanently kept.
//
// A3 (exception path — neither commit() nor rollback() called):
//   An exception causes the stack to unwind; the guard's destructor is invoked
//   automatically.  The destructor checks !committed_, which is true (neither
//   function was called), so it restores state_ = saved_.  The rollback is
//   automatic and requires no explicit action from the caller.
//
// A4 (reference lifetime):
//   state_ is a reference, not a copy.  The RobotState object that was passed
//   to the constructor MUST outlive the ScopedStateGuard.  The guard is
//   typically a local (stack) variable, and RobotState has a broader scope
//   (e.g., it lives on the caller's stack or as a long-lived member).
//   The caller is responsible for this contract; violating it produces
//   undefined behaviour (dangling reference).
//
// DESIGN QUESTION — throwing from a destructor during stack unwinding:
//   If a destructor throws while the stack is already unwinding due to an
//   exception, C++ calls std::terminate() (§15.5.1 [except.terminate]).
//   The program is terminated immediately; no further cleanup runs.
//   The practical approach in safety-critical robotics software is:
//     NEVER let a destructor propagate an exception.
//   Wrap any potentially throwing operation in try { ... } catch (...) {},
//   log the error internally if possible, and then continue — so the original
//   exception can propagate normally and the rest of the cleanup chain fires.
//   Marking destructors noexcept (the implicit default since C++11) makes the
//   compiler enforce this: if an exception escapes, std::terminate is called.

#include <iostream>
#include <chrono>
#include <stdexcept>
#include <string>

struct RobotState {
    double x, y, theta, v, omega;
};

// Helper — prints a state in the format used by main().
static void print_state(const char* label, const RobotState& s) {
    std::cout << label << ": x=" << s.x << " y=" << s.y << " theta=" << s.theta << "\n";
}

// ── class ScopedStateGuard ───────────────────────────────────────────────────

class ScopedStateGuard {
public:
    // Constructor — save current state and announce it.
    explicit ScopedStateGuard(RobotState& state)
        : state_(state), saved_(state), committed_(false)
    {
        std::cout << "state saved: x=" << saved_.x
                  << " y=" << saved_.y
                  << " theta=" << saved_.theta << "\n";
    }

    // Destructor — automatically restores if neither commit() nor rollback()
    // was called.  Must not throw: any exception from the assignment is caught
    // and silently discarded to avoid termination during stack unwinding.
    ~ScopedStateGuard() {
        if (!committed_) {
            try {
                state_ = saved_;
            } catch (...) {
                // Swallow: throwing from a destructor during stack unwinding
                // would call std::terminate().  Log here in production code.
            }
        }
    }

    // commit() — keep the modified state.  Idempotent: calling it multiple
    // times is safe because it only writes true to a bool.
    void commit() {
        committed_ = true;
    }

    // rollback() — restore the state immediately and prevent the destructor
    // from doing a second (redundant but potentially harmful) restore.
    // Idempotent: if already committed_ (e.g. called after commit()), the
    // assignment is skipped.
    void rollback() {
        if (!committed_) {
            state_ = saved_;
        }
        committed_ = true;
    }

    // Non-copyable, non-movable — a guard owns an exclusive reference to a
    // single piece of state and must not be duplicated or transferred.
    ScopedStateGuard(const ScopedStateGuard&)            = delete;
    ScopedStateGuard& operator=(const ScopedStateGuard&) = delete;
    ScopedStateGuard(ScopedStateGuard&&)                 = delete;
    ScopedStateGuard& operator=(ScopedStateGuard&&)      = delete;

private:
    RobotState& state_;   // reference to the actual state being guarded
    RobotState  saved_;   // copy taken at construction time
    bool        committed_;
};

// ── class ScopedPlanningTimer ────────────────────────────────────────────────

class ScopedPlanningTimer {
public:
    // Constructor — record the start time and store the label.
    explicit ScopedPlanningTimer(const std::string& label)
        : label_(label), start_(std::chrono::steady_clock::now())
    {}

    // Destructor — compute elapsed milliseconds and print the result.
    // Uses duration_cast so the value is always an integer (whole milliseconds).
    ~ScopedPlanningTimer() {
        auto elapsed = std::chrono::steady_clock::now() - start_;
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
        std::cout << "[timer] " << label_ << " took " << ms << " ms\n";
    }

    ScopedPlanningTimer(const ScopedPlanningTimer&)            = delete;
    ScopedPlanningTimer& operator=(const ScopedPlanningTimer&) = delete;

private:
    std::string label_;
    std::chrono::steady_clock::time_point start_;
};

// ── Helper: simulated motion planner step ────────────────────────────────────
static void apply_motion(RobotState& s, double dx, double dtheta) {
    s.x     += dx;
    s.theta += dtheta;
}

// ── main ─────────────────────────────────────────────────────────────────────
int main() {
    RobotState robot{0.0, 0.0, 0.0, 0.0, 0.0};

    // ── Rollback path ────────────────────────────────────────────────────────
    {
        ScopedStateGuard guard(robot);
        apply_motion(robot, 1.0, 0.5);
        print_state("after move", robot);
        guard.rollback();
        // destructor fires here — committed_ is true, no second restore
    }
    print_state("rolled back", robot);

    // ── Commit path ──────────────────────────────────────────────────────────
    {
        ScopedStateGuard guard(robot);
        apply_motion(robot, 1.0, 0.5);
        print_state("after move", robot);
        guard.commit();
        // destructor fires — committed_ is true, state is kept
    }
    print_state("committed — state kept", robot);

    // ── Timer path ───────────────────────────────────────────────────────────
    {
        ScopedPlanningTimer timer("planning_attempt");
        // minimal work — timer will report ~0 ms
        volatile int sink = 0;
        for (int i = 0; i < 1000; ++i) sink += i;
    }

    // ── Exception path (neither commit nor rollback) ──────────────────────────
    // Reset robot first
    robot = {0.0, 0.0, 0.0, 0.0, 0.0};
    try {
        ScopedStateGuard guard(robot);
        apply_motion(robot, 5.0, 3.14);
        // Something goes wrong — exception escapes without commit.
        throw std::runtime_error("planner failed");
        guard.commit();  // never reached
    } catch (const std::exception&) {
        // guard destructor has already fired and restored the state
    }
    print_state("exception path: state restored to", robot);

    return 0;
}
