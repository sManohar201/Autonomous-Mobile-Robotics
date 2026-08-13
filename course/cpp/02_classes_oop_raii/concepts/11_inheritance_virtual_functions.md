# 2.10 Inheritance & Virtual Functions

Consider a sensor fusion node that reads from wheel odometry, a GPS, and an IMU. Each sensor produces a pose estimate but they all work differently internally. Without inheritance you'd write separate code paths for each:

```cpp
void update(double dt) {
    if (source_type == "wheel")  { update_wheel(dt); }
    else if (source_type == "gnss") { update_gnss(dt); }
    else if (source_type == "imu")  { update_imu(dt); }
    // add a new sensor → edit this function
}
```

Every time you add a sensor you modify existing code. Fragile, and it grows without bound.

Inheritance solves this by letting you define a common interface (`LocalizationSource`) and have each sensor implement it. The fusion loop doesn't need to know which sensor it's talking to:

```cpp
for (auto& source : sources) {
    source->update(dt);   // correct implementation called automatically
}
```

**Virtual functions** are what make this work — they tell the compiler "don't decide which `update()` to call at compile time, decide at runtime based on the actual object type."

---

### Defining the Interface (Abstract Base Class)

```cpp
#include <iostream>
#include <string>
#include <memory>
#include <vector>

// Abstract base class — defines what every localization source must be able to do.
// You cannot create a LocalizationSource directly; it only exists to be inherited.
class LocalizationSource {
public:
    // Virtual destructor — explained in detail below.
    virtual ~LocalizationSource() = default;

    // Pure virtual (= 0): no implementation here.
    // Every derived class MUST provide its own version or it won't compile.
    virtual void   update(double dt) = 0;
    virtual std::string name() const = 0;

    // Virtual with a default: derived classes CAN override but don't have to.
    virtual bool is_healthy() const { return true; }

    // Non-virtual: shared behaviour, same for every source, not overridable.
    void log_status() const {
        std::cout << "[" << name() << "] healthy=" << is_healthy() << "\n";
    }
};

// This would be a compile error — LocalizationSource has pure virtual functions.
// LocalizationSource src;
```

---

### Implementing the Interface (Derived Classes)

```cpp
class WheelOdometry : public LocalizationSource {
public:
    explicit WheelOdometry(double wheel_radius)
        : wheel_radius_{wheel_radius} {}

    // 'override' tells the compiler: "I intend to override a virtual function."
    // If you mistype the name or get the signature wrong, the compiler errors —
    // without 'override' it would silently create a brand-new unrelated function.
    void update(double dt) override {
        std::cout << "WheelOdometry: integrating encoders, dt=" << dt << "\n";
    }

    std::string name() const override { return "WheelOdometry"; }

    // is_healthy() not overridden — uses base class default (returns true)

private:
    double wheel_radius_;
};

class GnssLocalization final : public LocalizationSource {
    // 'final' means nothing can inherit from GnssLocalization.
    // Useful when you know the hierarchy ends here — lets the compiler optimise.
public:
    void update(double dt) override {
        std::cout << "GnssLocalization: parsing NMEA fix, dt=" << dt << "\n";
    }

    std::string name() const override { return "GnssLocalization"; }

    // Override is_healthy() — GNSS is only healthy if we have a good fix.
    bool is_healthy() const override { return fix_quality_ > 1; }

private:
    int fix_quality_ = 2;
};
```

---

### Virtual Dispatch at Runtime

This is the payoff. The fusion loop holds a list of base-class pointers and calls `update()` on each — the right implementation runs automatically:

```cpp
void run_fusion_loop() {
    std::vector<std::unique_ptr<LocalizationSource>> sources;
    sources.push_back(std::make_unique<WheelOdometry>(0.1));
    sources.push_back(std::make_unique<GnssLocalization>());

    double dt = 0.05;
    for (auto& src : sources) {
        src->update(dt);     // WheelOdometry::update OR GnssLocalization::update
        src->log_status();   // always LocalizationSource::log_status (non-virtual)
    }
    // Output:
    // WheelOdometry: integrating encoders, dt=0.05
    // [WheelOdometry] healthy=1
    // GnssLocalization: parsing NMEA fix, dt=0.05
    // [GnssLocalization] healthy=1
}
```

Adding a new sensor (`ImuLocalization`) requires zero changes to `run_fusion_loop` — just write the new derived class.

---

### How It Works: the vtable

When a class has at least one virtual function, the compiler adds a hidden pointer (`vptr`) to every object. It points to a table of function pointers specific to that type:

```
WheelOdometry object in memory:
┌──────────────┐
│ vptr ────────┼──→ WheelOdometry vtable
│ wheel_radius_│         [0] → WheelOdometry::update()
└──────────────┘         [1] → WheelOdometry::name()
                         [2] → LocalizationSource::is_healthy()  (not overridden)

GnssLocalization object in memory:
┌──────────────┐
│ vptr ────────┼──→ GnssLocalization vtable
│ fix_quality_ │         [0] → GnssLocalization::update()
└──────────────┘         [1] → GnssLocalization::name()
                         [2] → GnssLocalization::is_healthy()    (overridden)
```

`src->update(dt)` follows the `vptr`, looks up slot `[0]`, calls whatever is there. One extra pointer dereference — negligible cost in practice.

---

### Virtual Destructor

Without a virtual destructor, deleting a derived object through a base pointer only runs the base destructor — the derived destructor is skipped. Resources owned by the derived class leak:

```cpp
// BAD — LocalizationSource has no virtual destructor
LocalizationSource* src = new WheelOdometry(0.1);
delete src;   // only ~LocalizationSource() runs — ~WheelOdometry() is SKIPPED
              // any resources WheelOdometry owns are leaked
              // undefined behaviour
```

With `virtual ~LocalizationSource() = default`, the vtable routes `delete` to the right destructor:

```cpp
// GOOD
LocalizationSource* src = new WheelOdometry(0.1);
delete src;   // virtual dispatch → ~WheelOdometry() runs first, then ~LocalizationSource()
```

**Rule:** if a class has any virtual functions, give it a virtual destructor.

---

### The Slicing Problem

Slicing happens when you copy a derived object into a base object **by value** — the derived-class data and vtable are chopped off:

```cpp
WheelOdometry wo{0.1};

LocalizationSource sliced = wo;   // SLICING: only the LocalizationSource part is copied
                                  // wheel_radius_ is gone, vptr points to base vtable
sliced.update(0.05);              // compile error — update() is pure virtual in base
```

Always use a pointer or reference when working polymorphically:

```cpp
LocalizationSource& ref = wo;    // no slicing — ref looks at the real WheelOdometry
LocalizationSource* ptr = &wo;   // same

ref.update(0.05);   // calls WheelOdometry::update — correct
```

---

### Access Specifiers in Inheritance

```cpp
struct Base {
    int pub  = 1;   // public
protected:
    int prot = 2;   // accessible in derived classes, not from outside
private:
    int priv = 3;   // never accessible in derived classes
};

struct D_pub   : public    Base {};  // pub stays public,    prot stays protected
struct D_prot  : protected Base {};  // pub becomes protected, prot stays protected
struct D_priv  : private   Base {};  // pub becomes private,  prot becomes private
```

In robotics code you almost always use `public` inheritance — it means "is-a". Protected and private inheritance mean "implemented-in-terms-of" and are rarely needed.

---

**Key takeaway:** Define a pure virtual interface in the base class, implement it in derived classes, and always use `override`. Store polymorphic objects as `unique_ptr<Base>` to avoid slicing. Give every polymorphic base class a virtual destructor or derived-class resources will leak silently.
