# Q10 — Static Member Definition Bug

This code compiles but fails to link. What is wrong and how do you fix it?

```cpp
// robot.hpp
#pragma once
#include <string>
#include <iostream>

class Robot {
public:
    explicit Robot(std::string name) : name_{std::move(name)} {
        ++count_;
    }
    ~Robot() { --count_; }

    static int count() { return count_; }

private:
    std::string name_;
    static int count_;   // declaration only
};

// main.cpp
#include "robot.hpp"

int main() {
    Robot r1{"r2d2"};
    Robot r2{"c3po"};
    std::cout << Robot::count() << "\n";
}
```

<details>
<summary>Answer</summary>

**The bug:** `static int count_` inside the class body is a **declaration**, not a definition. The linker cannot find the storage for `count_` because it has never been defined. The error is typically:

```
undefined reference to `Robot::count_'
```

**Fix:** Add the definition in exactly one `.cpp` file (outside the class):

```cpp
// robot.cpp
#include "robot.hpp"

int Robot::count_ = 0;   // definition — allocates storage and sets initial value
```

**Alternative (C++17 and later):** Declare it `inline static` inside the class, which serves as both declaration and definition:

```cpp
inline static int count_ = 0;   // C++17 — no separate .cpp definition needed
```

Or use `static constexpr` for compile-time constants, which has also been inline-defined since C++17:

```cpp
static constexpr int kMaxRobots = 10;   // inline definition, no .cpp needed
```

</details>
