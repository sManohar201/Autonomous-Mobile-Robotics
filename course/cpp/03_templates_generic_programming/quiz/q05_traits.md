# Q5 - Traits

What is a traits class, and why is it useful in generic robotics code?

<details>
<summary>Answer</summary>

A traits class stores type-specific metadata or behavior outside the type itself.

```cpp
template <typename Msg>
struct SensorTraits;
```

Specializations can define facts such as default topic, expected rate, frame id, or whether a message has covariance. Generic code can then use `SensorTraits<Msg>` without hard-coding message-specific logic into the algorithm.

</details>
