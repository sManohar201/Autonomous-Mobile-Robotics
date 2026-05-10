# Module 8 - Robotics Patterns

**Prerequisites:** Modules 1-7
**Goal:** Combine C++ language tools into robotics software architecture patterns: observers, plugins, state machines, component storage, and localization pipelines.

---

## 8.1 Patterns Are Tradeoffs

Patterns are not decorations. A pattern is useful when it makes ownership, extension points, data flow, or invariants clearer. Robotics code should prefer simple designs until the pressure for a pattern is real.

---

## 8.2 Eigen and Linear Algebra Boundaries

Production robotics often uses Eigen for vectors, matrices, transforms, and decompositions. Keep math types near math-heavy code and avoid leaking implementation details through every API. Even without Eigen, the boundary idea matters: pass typed poses, states, and covariances rather than raw arrays.

---

## 8.3 Observer Pattern

The observer pattern broadcasts events to subscribers:

```cpp
class SensorBus {
public:
    using Callback = std::function<void(const ImuSample&)>;
    void subscribe(Callback cb);
    void publish(const ImuSample& sample);
};
```

Use it for diagnostics, logging, and event fan-out. Be careful with callback lifetime and lock scope.

---

## 8.4 Plugin and Factory Architecture

A plugin interface defines behavior. A factory creates concrete implementations from configuration.

```cpp
class Localizer {
public:
    virtual ~Localizer() = default;
    virtual void predict(double dt) = 0;
    virtual void update(double z) = 0;
};
```

Factories centralize construction and keep user code independent of concrete types.

---

## 8.5 Finite State Machines

State machines are a good fit for drivers and lifecycle components: unconfigured, inactive, active, faulted. Represent states and events with `enum class`, and put transitions in one testable function.

---

## 8.6 Component Entity Systems

Entity-component style separates identity from data:

- Entity: ID
- Component: data attached to an entity
- System: logic operating on matching components

This can keep simulation or world-model code modular.

---

## 8.7 Mini Localization Architecture

A small localization library usually needs:

- State representation
- Prediction model
- Measurement update
- History/logging
- Reset and diagnostics
- Clear invariants

The best architecture is boring: typed data, explicit ownership, testable transitions, and measured performance.

---

## 8.8 Practical Rules

- Use observer for event fan-out, not core ownership.
- Use factories for runtime selection of known implementations.
- Use FSMs for lifecycle and recovery.
- Use component storage when many entities share optional data.
- Keep math invariants close to the types that own the math.

