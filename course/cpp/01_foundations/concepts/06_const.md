# 1.5 `const`

### `const` variables
```cpp
const double GRAVITY = 9.81;
GRAVITY = 10.0;   // compile error
```

### `const` with pointers — two distinct meanings
```cpp
int x = 5, y = 10;

const int* p1 = &x;   // pointer to const int
*p1 = 20;             // error: cannot modify value
p1  = &y;             // OK: can point elsewhere

int* const p2 = &x;   // const pointer to int
*p2 = 20;             // OK: can modify value
p2  = &y;             // error: cannot repoint
```

Read right-to-left: `const int* p` = "p is a pointer to const int".

### `const` references for function parameters
```cpp
void process(const std::vector<double>& data) { ... }  // no copy
```

Rule: pass cheap types (int, double) by value. Pass everything else by `const&`.

### `const` member functions
```cpp
class Sensor {
    double value_;
public:
    double read() const { return value_; }   // does not modify object
    void   set(double v) { value_ = v; }
};
```

Mark every read-only member function `const`. The compiler enforces it.
