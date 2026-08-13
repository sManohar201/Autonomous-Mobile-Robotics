# Q3 - Lambda Captures

Why is `[&]` risky in a callback that may run later?

<details>
<summary>Answer</summary>

`[&]` captures all used local variables by reference. If the callback is stored and invoked after the surrounding function returns, those references dangle. This is common in asynchronous robotics code: timers, subscriptions, futures, and thread pools.

Prefer explicit captures and make lifetime obvious. Capture values when possible; capture `this` only when object lifetime is guaranteed.

</details>
