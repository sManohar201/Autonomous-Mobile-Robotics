# Q4 - Lock Scope

Why should callbacks not usually be invoked while holding a mutex?

<details><summary>Answer</summary>

Callbacks may call back into your object, block, throw, or acquire other locks. Holding your mutex during callback execution increases deadlock and latency risk.

</details>
