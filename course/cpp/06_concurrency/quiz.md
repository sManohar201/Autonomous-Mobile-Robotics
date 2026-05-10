# Module 6 Quiz - Concurrency

## Q1 - Joinable Threads

What happens if a joinable `std::thread` is destroyed?

<details><summary>Answer</summary>

The program calls `std::terminate`. You must `join()` or `detach()` before destruction. In most robotics code, prefer `join()` for controlled shutdown.

</details>

---

## Q2 - Condition Variable Predicate

Why should `cv.wait(lock, predicate)` be preferred over plain `cv.wait(lock)`?

<details><summary>Answer</summary>

Threads can wake spuriously, and notifications can happen before the waiter starts waiting. The predicate rechecks the actual condition under the lock.

</details>

---

## Q3 - Atomic Limits

Why is `std::atomic<bool>` not enough to protect a queue?

<details><summary>Answer</summary>

The queue has compound state: storage, size, head/tail, and invariants across those fields. A mutex is needed to protect those multi-step updates.

</details>

---

## Q4 - Lock Scope

Why should callbacks not usually be invoked while holding a mutex?

<details><summary>Answer</summary>

Callbacks may call back into your object, block, throw, or acquire other locks. Holding your mutex during callback execution increases deadlock and latency risk.

</details>

---

## Q5 - Bounded Queue Policy

Why should a bounded queue define its overload behavior explicitly?

<details><summary>Answer</summary>

Robotics systems need predictable behavior under load. The queue should clearly block, reject new items, drop oldest, or drop newest rather than growing unbounded and failing later.

</details>

