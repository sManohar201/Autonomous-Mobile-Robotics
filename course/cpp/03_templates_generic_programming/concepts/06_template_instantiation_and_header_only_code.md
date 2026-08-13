# 3.5 Template Instantiation and Header-Only Code

Templates are usually implemented in headers because the compiler must see the full definition at the point of instantiation.

```cpp
// math.hpp
template <typename T>
T norm2(T x, T y) {
    return x * x + y * y;
}
```

If you put a template definition only in a `.cpp` file, another translation unit cannot instantiate it unless you explicitly instantiate the needed types:

```cpp
template double norm2<double>(double, double);
template float norm2<float>(float, float);
```

Most teaching code and many robotics utility libraries keep templates header-only for simplicity. Production libraries may use explicit instantiation to reduce compile times and binary size.
