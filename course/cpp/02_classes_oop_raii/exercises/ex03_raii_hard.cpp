// Exercise 03 — RAII with commit/rollback semantics: ScopedStateGuard
//
// Context
// -------
// Motion planners tentatively apply state transitions and later decide whether
// to keep or discard them.  A guard object that automatically rolls back on
// scope exit — unless explicitly committed — is the safest way to express this.
// std::lock_guard uses the same pattern.
//
// The pattern is sometimes called "scope guard" or "transactional scope."
// In production robotics code (e.g., nav2, MoveIt) you will find this idiom
// everywhere a function can fail partway through and must leave shared state
// consistent.
//
// ── RobotState ───────────────────────────────────────────────────────────────
// Provided below.  Do NOT modify it.
//
//   struct RobotState { double x, y, theta, v, omega; };
//
// ── ScopedStateGuard ─────────────────────────────────────────────────────────
// Constructor: ScopedStateGuard(RobotState& state)
//   Saves a copy of state (saved_ = state).
//   Prints "state saved: x=<x> y=<y> theta=<theta>\n".
//
// commit():
//   Sets committed_ = true.
//   Idempotent: calling commit() twice must be safe and have no side effect.
//
// rollback():
//   Restores state_ from saved_ immediately.
//   Sets committed_ = true so the destructor does NOT restore again.
//   Idempotent: calling rollback() twice must be safe.
//   THINK: why must rollback() set committed_ = true after restoring?
//     If you don't, the destructor will restore a second time — which is
//     a no-op in this simple case but would be catastrophic with resources.
//
// Destructor:
//   If !committed_, restore state_ from saved_.
//   DESIGN QUESTION (answer in a comment below):
//     What if the restore operation could throw?  For example, if RobotState
//     contained a member that has a throwing copy assignment.  Should the
//     destructor propagate the exception or swallow it?  What does the C++
//     standard say about throwing from a destructor during stack unwinding?
//     What is the practical approach in safety-critical robotics software?
//
// Non-copyable, non-movable: delete all four copy/move operations.
//
// ── ScopedPlanningTimer ──────────────────────────────────────────────────────
// Measures elapsed time for a labelled planning attempt.
//
// Constructor: ScopedPlanningTimer(const std::string& label)
//   Records std::chrono::steady_clock::now() as start_.
//   Stores label_.
//
// Destructor:
//   Computes elapsed milliseconds.
//   Prints "[timer] <label> took <ms> ms\n".
//
// Non-copyable (delete copy constructor and copy assignment).
// Move is allowed but not tested here.
//
// ── Design question — answer before coding ───────────────────────────────────
//
// Q1. If rollback() is called and THEN the destructor runs (normal case after
//     explicit rollback), what sequence of events occurs?  Trace through the
//     flags.
//
// Q2. If commit() is called and THEN the destructor runs, what happens?
//
// Q3. If neither commit() nor rollback() is called (exception path), what
//     happens?
//
// Q4. The guard stores state_ as a reference.  What does this mean for the
//     lifetime of the RobotState object relative to the guard?  Who is
//     responsible for ensuring the referent outlives the guard?
//
// ── Expected output ──────────────────────────────────────────────────────────
//   state saved: x=0 y=0 theta=0
//   after move: x=1 y=0 theta=0.5
//   rolled back: x=0 y=0 theta=0
//   state saved: x=0 y=0 theta=0
//   after move: x=1 y=0 theta=0.5
//   committed — state kept: x=1 y=0 theta=0.5
//   [timer] planning_attempt took ~0 ms
//   exception path: state restored to x=0 y=0 theta=0

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

// ── ANSWER Q1–Q4 here before implementing ────────────────────────────────────
// A1 (rollback() then destructor):
//   TODO
//
// A2 (commit() then destructor):
//   TODO
//
// A3 (exception path, neither called):
//   TODO
//
// A4 (reference lifetime):
//   TODO
//
// DESIGN QUESTION — throwing from destructor during unwinding:
//   TODO: explain the C++ rule (std::terminate) and the practical approach.

// ── class ScopedStateGuard ───────────────────────────────────────────────────

class ScopedStateGuard {
public:
    // TODO: Constructor — store reference, save current state, print.
    explicit ScopedStateGuard(RobotState& state);

    // TODO: Destructor — restore if !committed_.
    //   Do NOT throw.  Swallow any exception from the assignment and log it.
    ~ScopedStateGuard();

    // TODO: commit() — set committed_ = true.  Idempotent.
    void commit();

    // TODO: rollback() — restore state immediately, set committed_ = true.
    //   Idempotent (safe to call multiple times).
    void rollback();

    // Non-copyable, non-movable.
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
    // TODO: Constructor — record start time, store label.
    explicit ScopedPlanningTimer(const std::string& label);

    // TODO: Destructor — compute and print elapsed ms.
    ~ScopedPlanningTimer();

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
