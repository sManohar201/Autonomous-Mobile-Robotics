# 4.7 `std::function`

`std::function` stores any callable with a matching signature.

```cpp
std::function<void(double)> update_callback;

update_callback = [](double dt) {
    std::cout << "dt=" << dt << "\n";
};
```

It is useful for callback registration and plugin-like seams:

```cpp
class Timer {
public:
    using Callback = std::function<void(double)>;
    explicit Timer(Callback cb) : callback_{std::move(cb)} {}

    void tick(double dt) { callback_(dt); }

private:
    Callback callback_;
};
```

Tradeoff: `std::function` may allocate and performs type-erased dispatch. For hot inner loops, prefer templates or direct function objects.
