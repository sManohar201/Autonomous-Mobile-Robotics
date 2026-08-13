# Q2 - Condition Variable Predicate

Why should `cv.wait(lock, predicate)` be preferred over plain `cv.wait(lock)`?

<details><summary>Answer</summary>

Threads can wake spuriously, and notifications can happen before the waiter starts waiting. The predicate rechecks the actual condition under the lock.

</details>
