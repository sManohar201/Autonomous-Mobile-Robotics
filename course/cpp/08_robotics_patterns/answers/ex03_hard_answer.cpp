#include <cassert>
#include <iostream>
#include <string_view>

enum class State { Unconfigured, Inactive, Active, Fault };
enum class Event { Configure, Activate, Deactivate, Error, Reset };

struct Transition {
    State state;
    bool accepted;
    std::string_view reason;
};

constexpr Transition transition(State s, Event e) {
    if (e == Event::Error) return {State::Fault, true, "fault"};
    switch (s) {
    case State::Unconfigured:
        return e == Event::Configure ? Transition{State::Inactive, true, "configured"}
                                     : Transition{s, false, "must configure first"};
    case State::Inactive:
        return e == Event::Activate ? Transition{State::Active, true, "active"}
                                    : Transition{s, false, "not accepted"};
    case State::Active:
        return e == Event::Deactivate ? Transition{State::Inactive, true, "inactive"}
                                      : Transition{s, false, "not accepted"};
    case State::Fault:
        return e == Event::Reset ? Transition{State::Unconfigured, true, "reset"}
                                 : Transition{s, false, "faulted"};
    }
    return {State::Fault, false, "unreachable"};
}

int main() {
    static_assert(transition(State::Unconfigured, Event::Configure).state == State::Inactive);
    static_assert(!transition(State::Unconfigured, Event::Activate).accepted);
    assert(transition(State::Active, Event::Error).state == State::Fault);
    std::cout << "ex03_hard_answer passed\n";
}

