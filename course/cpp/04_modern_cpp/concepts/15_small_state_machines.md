# 4.14 Small State Machines

State machines appear in robot behavior, sensor drivers, lifecycle nodes, and recovery logic.

```cpp
enum class State {
    Idle,
    Armed,
    Running,
    Fault
};

enum class Event {
    Configure,
    Start,
    Stop,
    Error
};
```

A transition function makes behavior explicit:

```cpp
constexpr State transition(State s, Event e) {
    switch (s) {
    case State::Idle:
        return e == Event::Configure ? State::Armed : s;
    case State::Armed:
        return e == Event::Start ? State::Running : s;
    case State::Running:
        return e == Event::Stop ? State::Armed : s;
    case State::Fault:
        return State::Fault;
    }
    return State::Fault;
}
```

If the transition function is `constexpr`, you can test behavior at compile time:

```cpp
static_assert(transition(State::Idle, Event::Configure) == State::Armed);
```

For large behavior trees or lifecycle graphs, use dedicated architecture. For small deterministic components, a simple typed state machine is often clearer and safer.
