# 3.12 CRTP

CRTP is the Curiously Recurring Template Pattern: a base class template takes the derived class as a template parameter.

```cpp
template <typename Derived>
class Printable {
public:
    void print() const {
        static_cast<const Derived&>(*this).print_impl();
    }
};

class Pose2d : public Printable<Pose2d> {
public:
    void print_impl() const {
        std::cout << "Pose2d\n";
    }
};
```

This gives compile-time polymorphism. There is no virtual table and no dynamic dispatch.

Use CRTP when:

- The set of derived types is known at compile time
- You need zero-overhead static dispatch
- You want to share interface glue without runtime polymorphism

Do not use CRTP when you need heterogeneous runtime collections like `std::vector<std::unique_ptr<SensorBase>>`. That is Module 2 runtime polymorphism territory.
