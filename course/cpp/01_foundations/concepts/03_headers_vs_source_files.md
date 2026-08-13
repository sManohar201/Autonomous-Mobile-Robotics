# 1.2 Headers vs Source Files

| File | Contains | Why |
|------|----------|-----|
| `.hpp` header | **Declarations** | Tells the compiler what exists |
| `.cpp` source | **Definitions** | The actual code, compiled once |

A **declaration** says "this exists, here is its type":
```cpp
double dot_product(double ax, double ay, double bx, double by);
```

A **definition** provides the body:
```cpp
double dot_product(double ax, double ay, double bx, double by) {
    return ax * bx + ay * by;
}
```

If you put a definition in a header and include it from two `.cpp` files, both `.o` files contain a copy — the linker sees two definitions of the same symbol and errors: `multiple definition of 'dot_product'`.

### What belongs in headers

```cpp
// OK in headers
class Sensor { ... };                              // class definition
double dot(double a, double b);                    // function declaration
inline double sq(double x) { return x*x; }        // inline function
template<typename T> T clamp(T v, T lo, T hi);    // template (must be in header)
constexpr double PI = 3.14159265358979;            // constexpr constant
```

```cpp
// NOT in headers (causes multiple definition)
double dot(double a, double b) { return a * b; }  // non-inline definition
int counter = 0;                                   // non-const variable
```
