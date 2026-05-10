// Exercise 05 Capstone - Config Parser + State Machine
//
// Goal:
//   Parse typed controller configuration and drive a small constexpr state
//   machine.

#include <cassert>
#include <map>
#include <optional>
#include <iostream>
#include <string>
#include <variant>

using ConfigValue = std::variant<int, double, bool, std::string>;
using ConfigMap = std::map<std::string, ConfigValue>;

struct ControllerConfig {
    double rate_hz;
    std::string frame_id;
    bool publish_debug;
};

// TODO 1: Implement get_as<T>(cfg, key).
// Return std::optional<T>. Missing key or wrong type returns std::nullopt.

// TODO 2: Implement parse_controller_config.
// Required:
//   - rate_hz: double, must be > 0
//   - frame_id: std::string, must be non-empty
// Optional:
//   - publish_debug: bool, defaults to false

enum class State {
    Idle,
    Configured,
    Running,
    Fault
};

enum class Event {
    Configure,
    Start,
    Stop,
    Error,
    Reset
};

// TODO 3: Implement constexpr transition(State, Event).
// Rules:
//   Idle + Configure -> Configured
//   Configured + Start -> Running
//   Running + Stop -> Configured
//   Any state + Error -> Fault
//   Fault + Reset -> Idle
//   Otherwise stay in current state.

int main() {
    ConfigMap cfg{
        {"rate_hz", 50.0},
        {"frame_id", std::string{"base_link"}},
        {"publish_debug", true},
    };

    // TODO: make these pass.
    // auto parsed = parse_controller_config(cfg);
    // assert(parsed);
    // assert(parsed->rate_hz == 50.0);
    // assert(parsed->frame_id == "base_link");
    // assert(parsed->publish_debug);
    //
    // static_assert(transition(State::Idle, Event::Configure) == State::Configured);
    // static_assert(transition(State::Configured, Event::Start) == State::Running);
    // static_assert(transition(State::Running, Event::Stop) == State::Configured);
    // static_assert(transition(State::Running, Event::Error) == State::Fault);
    // static_assert(transition(State::Fault, Event::Reset) == State::Idle);

    (void)cfg;

    std::cout << "ex05_capstone_config_state_machine passed\n";
}

