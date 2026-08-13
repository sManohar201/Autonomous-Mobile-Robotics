# Q10 - Compile-Time Units

Why is this valuable?

```cpp
using Meters = Quantity<MeterTag>;
using Seconds = Quantity<SecondTag>;
```

<details>
<summary>Answer</summary>

Both types may store a `double`, but they are distinct at compile time. A function expecting `Meters` cannot accidentally receive `Seconds`. This prevents unit bugs without runtime overhead.

The larger design goal is to make invalid physical operations unrepresentable or at least explicit.

</details>
