# 2.3 Member Initializer Lists

The **member initializer list** (MIL) runs before the constructor body. It is not just a style choice — for certain members it is the only option.

### Why MIL Is Preferred

```cpp
#include <string>
#include <iostream>

class Node {
public:
    // WRONG: const member cannot be assigned in the body.
    // Uncommenting the body-assignment version gives a compile error.
    //
    // Node(int id) {
    //     id_ = id;     // error: assignment to const member
    // }

    // RIGHT: const and reference members MUST be in the MIL.
    Node(int id, std::string name, double& shared_val)
        : id_{id}                    // const int — only place to set it
        , name_{std::move(name)}     // string: MIL calls move-ctor directly
        , ref_{shared_val}           // reference — must bind here
    {}

    void print() const {
        std::cout << "Node " << id_ << ": " << name_ << "\n";
    }

private:
    const int id_;          // MUST be in MIL
    std::string name_;      // prefer MIL (avoids default-then-copy)
    double& ref_;           // MUST be in MIL
};
```

### MIL Order Matches Declaration Order

The MIL initializes members in **declaration order**, not the order they appear in the MIL. Writing them out of order compiles but is misleading.

```cpp
// Safe: MIL order matches declaration order.
class Pose {
public:
    Pose(double x, double y, double yaw)
        : x_{x}, y_{y}, yaw_{yaw}   // same order as declared below — fine
    {}
private:
    double x_, y_, yaw_;
};

// DANGER: MIL order disagrees with declaration order.
// Members are always initialized in DECLARATION order, not MIL order.
// Here: sum_ is declared first, so it is initialized first —
// but its MIL entry uses x_ and y_, which haven't been initialized yet.
// Result: sum_ gets garbage. Compiler warns with -Wreorder.
class Stats {
public:
    Stats(double x, double y)
        : x_{x}, y_{y}, sum_{x_ + y_}   // BUG: sum_ runs before x_ and y_
    {}
    double sum() const { return sum_; }
private:
    double sum_;   // declared first → initialized first, even though last in MIL
    double x_;
    double y_;
};

// Fix: declare members in dependency order.
class StatsSafe {
public:
    StatsSafe(double x, double y)
        : x_{x}, y_{y}, sum_{x_ + y_}   // fine: x_ and y_ declared before sum_
    {}
private:
    double x_;
    double y_;
    double sum_;   // declared last → initialized last, x_ and y_ are ready
};
```

### Body Assignment Silently Failing

```cpp
// This does NOT compile — shown to illustrate the category of error.
//
// class BadNode {
// public:
//     BadNode(int id) {
//         id_ = id;    // error: cannot assign to const member 'id_'
//     }
// private:
//     const int id_;
// };
//
// The const member is default-initialized (undefined for int) before the
// body runs. By the time the body assignment executes, it's too late —
// the object is already "constructed" with an indeterminate id_.
```

**Key takeaway:** Always initialize members in the MIL, in declaration order. `const` and reference members have no alternative — the compiler enforces it.
