# Q4 - Non-Type Template Parameters

Why is `Vector<double, 3>` safer than a vector class that stores its dimension as a runtime integer?

<details>
<summary>Answer</summary>

The dimension is part of the type. A function that expects `Vector<double, 3>` cannot accidentally receive `Vector<double, 2>`. This catches dimension mismatch at compile time instead of producing a runtime error or, worse, a silent math bug.

This is useful for robotics math: 3D acceleration, 6D state vectors, 6x6 covariance matrices, and fixed-capacity embedded buffers.

</details>
