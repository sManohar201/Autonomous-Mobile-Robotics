# 1.7 Function Overloading

Multiple functions with the same name, different parameter types. The compiler picks at compile time — no runtime cost.

```cpp
double magnitude(double x, double y)           { return std::sqrt(x*x + y*y); }
double magnitude(double x, double y, double z) { return std::sqrt(x*x + y*y + z*z); }

magnitude(3.0, 4.0);          // calls 2D version
magnitude(1.0, 2.0, 2.0);     // calls 3D version
```

Compiler prefers exact match over implicit conversion. If two conversions are equally good: ambiguous → compile error.
