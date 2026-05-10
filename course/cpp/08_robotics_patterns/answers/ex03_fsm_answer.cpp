#include <cassert>
#include <iostream>

enum class LifecycleState {
    Unconfigured,
    Inactive,
    Active,
    Fault
};

enum class LifecycleEvent {
    Configure,
    Activate,
    Deactivate,
    Error,
    Reset
};

constexpr LifecycleState transition(LifecycleState state, LifecycleEvent event) {
    if (event == LifecycleEvent::Error) return LifecycleState::Fault;
    switch (state) {
    case LifecycleState::Unconfigured:
        return event == LifecycleEvent::Configure ? LifecycleState::Inactive : state;
    case LifecycleState::Inactive:
        return event == LifecycleEvent::Activate ? LifecycleState::Active : state;
    case LifecycleState::Active:
        return event == LifecycleEvent::Deactivate ? LifecycleState::Inactive : state;
    case LifecycleState::Fault:
        return event == LifecycleEvent::Reset ? LifecycleState::Unconfigured : state;
    }
    return LifecycleState::Fault;
}

int main() {
    static_assert(transition(LifecycleState::Unconfigured, LifecycleEvent::Configure) ==
                  LifecycleState::Inactive);
    static_assert(transition(LifecycleState::Inactive, LifecycleEvent::Activate) ==
                  LifecycleState::Active);
    static_assert(transition(LifecycleState::Active, LifecycleEvent::Error) ==
                  LifecycleState::Fault);
    std::cout << "ex03_fsm_answer passed\n";
}

