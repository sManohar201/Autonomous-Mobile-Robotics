# Q7 — Abstract Class and Slicing

What does this program print, or does it fail to compile? Explain.

```cpp
#include <iostream>

class Shape {
public:
    virtual double area() const = 0;
    virtual ~Shape() = default;
    void describe() const {
        std::cout << "area=" << area() << "\n";
    }
};

class Square : public Shape {
public:
    explicit Square(double s) : side_{s} {}
    double area() const override { return side_ * side_; }
private:
    double side_;
};

void print_area(Shape s) {   // takes Shape by VALUE
    s.describe();
}

int main() {
    Square sq{4.0};
    print_area(sq);
}
```

<details>
<summary>Answer</summary>

**This does not compile.**

`print_area` takes a `Shape` by value. To construct the parameter, the compiler must copy a `Square` into a `Shape` object. But `Shape` has a pure virtual function (`area() = 0`), making it an **abstract class**. Abstract classes cannot be instantiated — you cannot create an object of type `Shape`. The compiler rejects the copy.

Additionally, even if `Shape` were concrete, this would be the **slicing problem**: copying a `Square` into a `Shape` would discard `side_` and the vtable pointer for `Square`, leaving a useless truncated object.

**Fix:** Pass by const reference or pointer to preserve polymorphism and avoid slicing:

```cpp
void print_area(const Shape& s) {
    s.describe();   // virtual dispatch works correctly through reference
}
```

</details>
