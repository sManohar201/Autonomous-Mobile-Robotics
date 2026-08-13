# 1.4 Basic Types

| Type | Size | Use |
|------|------|-----|
| `bool` | 1 byte | true / false |
| `int` | 4 bytes | general integer |
| `double` | 8 bytes | double-precision float — use for physics |
| `float` | 4 bytes | single-precision — use for sensor data |
| `size_t` | 8 bytes | sizes and indices |

### Fixed-width types (`<cstdint>`)
```cpp
#include <cstdint>
uint8_t   byte_val = 255;     // 0 to 255
int16_t   short_val = -1000;
uint32_t  count = 0;
int64_t   timestamp_ns = 0;   // nanoseconds since epoch
```

### Implicit conversion and narrowing
```cpp
double d = 3.99;
int    i = d;           // truncates to 3 — silent data loss
uint8_t b = 300;        // wraps to 44 — silent data loss

// Use explicit casts when you intend conversion:
int i2 = static_cast<int>(d);
```
