# 1.8 Namespaces

```cpp
namespace math {
    double dot(double ax, double ay, double bx, double by);
}

// Usage options:
double d = math::dot(1.0, 0.0, 0.0, 1.0);   // fully qualified — always safe

using math::dot;      // imports one name — OK inside a function
using namespace math; // imports everything — NEVER in a header
```

**Never `using namespace X;` in a header.** It pollutes every file that includes it.

### Anonymous namespaces — file-local linkage
```cpp
namespace {
    double clamp01(double x) { ... }  // invisible outside this .cpp
}
```
