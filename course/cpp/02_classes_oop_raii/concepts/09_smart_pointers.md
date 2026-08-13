# 2.8 Smart Pointers

Smart pointers are RAII wrappers around raw pointers. They live in `<memory>`. Prefer them over `new`/`delete` in all new code.

### `unique_ptr` — Sole Ownership

```cpp
#include <memory>
#include <iostream>
#include <string>

class Lidar {
public:
    explicit Lidar(std::string model) : model_{std::move(model)} {
        std::cout << "Lidar " << model_ << " online\n";
    }
    ~Lidar() { std::cout << "Lidar " << model_ << " offline\n"; }
    void spin() { std::cout << model_ << " spinning\n"; }

private:
    std::string model_;
};

void unique_ptr_demo() {
    // make_unique is preferred over new: exception-safe, clear ownership.
    auto lidar = std::make_unique<Lidar>("Velodyne VLP-16");

    lidar->spin();          // arrow operator: same as (*lidar).spin()
    (*lidar).spin();        // dereference also works

    // unique_ptr is NOT copyable.
    // auto lidar2 = lidar;   // compile error

    // unique_ptr IS movable — transfers sole ownership.
    auto lidar2 = std::move(lidar);
    // lidar is now null; lidar2 owns the object.

    if (!lidar) std::cout << "lidar is null after move\n";

    lidar2->spin();
}   // lidar2 destroyed here — Lidar destructor runs automatically
```

### `shared_ptr` — Shared Ownership

```cpp
#include <memory>
#include <iostream>

class Map {
public:
    Map() { std::cout << "Map created\n"; }
    ~Map() { std::cout << "Map destroyed\n"; }
};

void shared_ptr_demo() {
    std::shared_ptr<Map> map1 = std::make_shared<Map>();
    std::cout << "use_count=" << map1.use_count() << "\n";  // 1

    {
        std::shared_ptr<Map> map2 = map1;   // copy: ref count -> 2
        std::cout << "use_count=" << map1.use_count() << "\n";  // 2
        // Both map1 and map2 own the object.
    }   // map2 destroyed: ref count -> 1. Map NOT yet destroyed.

    std::cout << "use_count=" << map1.use_count() << "\n";  // 1
}   // map1 destroyed: ref count -> 0. Map destroyed here.
```

### `weak_ptr` — Non-Owning Observer

`weak_ptr` observes a `shared_ptr` without incrementing the reference count. Use it to break ownership cycles (e.g., parent/child graphs) or to hold a cached reference that might expire.

```cpp
#include <memory>
#include <iostream>

void weak_ptr_demo() {
    std::shared_ptr<int> strong = std::make_shared<int>(42);
    std::weak_ptr<int> weak = strong;   // no ref count increase

    std::cout << "use_count=" << strong.use_count() << "\n";  // 1

    // To use a weak_ptr you must lock() it — produces a temporary shared_ptr.
    if (auto locked = weak.lock()) {
        std::cout << "value=" << *locked << "\n";   // 42
    }

    strong.reset();   // release the shared ownership; object destroyed

    if (weak.expired()) {
        std::cout << "object has been destroyed\n";
    }
}
```

### When to Use Each

| Smart pointer | Use when |
|---|---|
| `unique_ptr` | Default choice. One clear owner. Zero overhead vs raw pointer. |
| `shared_ptr` | Genuinely shared ownership (multiple subsystems need the same object alive). |
| `weak_ptr` | You need to observe a `shared_ptr` without prolonging its life; breaking cycles. |
| Raw pointer `T*` | Non-owning observation of something guaranteed to outlive you (function parameters, iterators). |

```cpp
#include <memory>
#include <vector>
#include <iostream>

struct Plugin {
    virtual ~Plugin() = default;
    virtual void run() = 0;
};

struct EkfPlugin : Plugin {
    void run() override { std::cout << "EKF running\n"; }
};

// Typical robotics pattern: a manager owns plugins via unique_ptr.
class SensorFusion {
public:
    void add_plugin(std::unique_ptr<Plugin> p) {
        plugins_.push_back(std::move(p));
    }
    void run_all() {
        for (auto& p : plugins_) p->run();
    }

private:
    std::vector<std::unique_ptr<Plugin>> plugins_;
};

int main() {
    SensorFusion sf;
    sf.add_plugin(std::make_unique<EkfPlugin>());
    sf.run_all();
}
```

**Key takeaway:** Default to `unique_ptr`. Reach for `shared_ptr` only when you truly cannot determine a single owner. Never use `new`/`delete` in application code.
