# Q12 — Abstract Class Interface

Complete the `LocalizationFilter` abstract base class and one concrete derived class. Fill in the three marked sections.

```cpp
#include <iostream>
#include <string>
#include <memory>
#include <vector>

// Abstract base: defines the interface for all state estimators.
class LocalizationFilter {
public:
    // TODO 1: Declare a virtual destructor so derived classes
    //         are destroyed correctly through a base pointer.


    // TODO 2: Declare two pure virtual functions:
    //   - predict(double dt)  — propagates state forward by dt seconds
    //   - update(double meas) — incorporates a measurement
    //   Both take a double and return void.


    // Non-virtual utility: calls predict then update.
    void step(double dt, double meas) {
        predict(dt);
        update(meas);
    }

    virtual std::string name() const = 0;
};

// TODO 3: Implement a concrete class KalmanFilter that:
//   - inherits publicly from LocalizationFilter
//   - has a constructor taking (double process_noise, double meas_noise)
//   - overrides predict, update, and name
//   - stores a double state_ (initialised to 0.0) and prints what it's doing


// --- Driver (do not modify) ---
int main() {
    std::vector<std::unique_ptr<LocalizationFilter>> filters;
    filters.push_back(std::make_unique<KalmanFilter>(0.1, 0.5));

    for (auto& f : filters) {
        f->step(0.05, 3.14);
        std::cout << f->name() << " state processed\n";
    }
}
```

<details>
<summary>Answer</summary>

```cpp
#include <iostream>
#include <string>
#include <memory>
#include <vector>

class LocalizationFilter {
public:
    // TODO 1: Virtual destructor.
    virtual ~LocalizationFilter() = default;

    // TODO 2: Pure virtual functions.
    virtual void predict(double dt)    = 0;
    virtual void update(double meas)   = 0;

    void step(double dt, double meas) {
        predict(dt);
        update(meas);
    }

    virtual std::string name() const = 0;
};

// TODO 3: Concrete derived class.
class KalmanFilter : public LocalizationFilter {
public:
    KalmanFilter(double process_noise, double meas_noise)
        : process_noise_{process_noise}
        , meas_noise_{meas_noise}
        , state_{0.0}
    {}

    void predict(double dt) override {
        // Simplified: just drift state by a tiny amount.
        state_ += process_noise_ * dt;
        std::cout << "KalmanFilter predict dt=" << dt
                  << " state=" << state_ << "\n";
    }

    void update(double meas) override {
        // Simplified Kalman update: blend state toward measurement.
        double gain = process_noise_ / (process_noise_ + meas_noise_);
        state_ += gain * (meas - state_);
        std::cout << "KalmanFilter update meas=" << meas
                  << " state=" << state_ << "\n";
    }

    std::string name() const override { return "KalmanFilter"; }

private:
    double process_noise_;
    double meas_noise_;
    double state_;
};

int main() {
    std::vector<std::unique_ptr<LocalizationFilter>> filters;
    filters.push_back(std::make_unique<KalmanFilter>(0.1, 0.5));

    for (auto& f : filters) {
        f->step(0.05, 3.14);
        std::cout << f->name() << " state processed\n";
    }
}
```

**What to notice:**
- `virtual ~LocalizationFilter() = default` ensures that `delete` through a `LocalizationFilter*` calls `~KalmanFilter()` correctly.
- `= 0` on `predict` and `update` makes `LocalizationFilter` abstract — you cannot instantiate it directly.
- `override` on each derived method causes a compile error if the signature doesn't match a base virtual function.
- The vector stores `unique_ptr<LocalizationFilter>` — polymorphism via pointer, no slicing.

</details>
