# ROS2 Module 05 Quiz - Executors & Concurrency

## Q1

What does an executor do?

<details><summary>Answer</summary>

It waits for work and runs callbacks from subscriptions, timers, services, actions, and other waitables.

</details>

## Q2

When should a callback group be mutually exclusive?

<details><summary>Answer</summary>

When callbacks share mutable state and must not run at the same time.

</details>

## Q3

Why can a multi-threaded executor introduce bugs?

<details><summary>Answer</summary>

Callbacks can overlap, exposing data races, lock-order issues, and reentrancy problems.

</details>

## Q4

Why keep timer callbacks bounded?

<details><summary>Answer</summary>

Long timer callbacks delay other work and can introduce control latency or stale processing.

</details>

## Q5

What is a safe pattern for subscriber/timer interaction?

<details><summary>Answer</summary>

Subscriber callbacks update a small protected latest-sample cache. Timer callbacks copy a snapshot under lock, release the lock, then do heavier processing.

</details>

