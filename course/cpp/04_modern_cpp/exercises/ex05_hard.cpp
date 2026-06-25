// Exercise 05 (Hard) - Config Parser + constexpr Lifecycle FSM
//
// Tasks:
//   1. Parse ControllerConfig from typed map values.
//   2. Return detailed ParseResult rather than just optional.
//   3. Implement constexpr lifecycle transition.
//   4. Add static_assert transition tests for valid and invalid events.

#include <cassert>
#include <map>
#include <optional>
#include <iostream>
#include <string>
#include <variant>

using ConfigValue = std::variant<double, bool, std::string>;
using ConfigMap   = std::map<std::string, ConfigValue>;

struct ControllerConfig {
    double rate_hz;
    std::string frame_id;
    bool debug;
};

struct ParseResult {
    std::optional<ControllerConfig> config;
    std::string error;
};

template <typename T>
std::optional<T> get_as(const ConfigMap& cfg, const std::string& key) {
    auto it = cfg.find(key);
    if (it == cfg.end()) return std::nullopt;
    if (auto value = std::get_if<T>(&it->second)) return *value;
    return std::nullopt;
}

ParseResult parse_controller(const ConfigMap& cfg) {
    auto rate  = get_as<double>(cfg, "rate_hz");
    auto frame = get_as<std::string>(cfg, "frame_id");
    if (!rate || *rate <= 0.0)    return {std::nullopt, "invalid rate_hz"};
    if (!frame || frame->empty()) return {std::nullopt, "invalid frame_id"};
    return {ControllerConfig{*rate, *frame, get_as<bool>(cfg, "debug").value_or(false)}, ""};
}

enum class State { Idle, Configured, Running, Fault };
enum class Event { Configure, Start, Stop, Error, Reset };

constexpr State transition(State s, Event e) {
    if (e == Event::Error) return State::Fault;
    switch (s) {
    case State::Idle:       return e == Event::Configure ? State::Configured : s;
    case State::Configured: return e == Event::Start     ? State::Running    : s;
    case State::Running:    return e == Event::Stop      ? State::Configured : s;
    case State::Fault:      return e == Event::Reset     ? State::Idle       : s;
    }
    return State::Fault;
}

int main() {
    ConfigMap cfg{{"rate_hz", 20.0}, {"frame_id", std::string{"base_link"}}};
    auto result = parse_controller(cfg);
    assert(result.config);
    assert(result.error.empty());
    assert(result.config->rate_hz == 20.0);

    ConfigMap bad{{"rate_hz", 0.0}, {"frame_id", std::string{"base_link"}}};
    auto bad_result = parse_controller(bad);
    assert(!bad_result.config);
    assert(bad_result.error == "invalid rate_hz");

    static_assert(transition(State::Idle, Event::Configure)   == State::Configured);
    static_assert(transition(State::Configured, Event::Start) == State::Running);
    static_assert(transition(State::Running, Event::Stop)     == State::Configured);
    static_assert(transition(State::Running, Event::Error)    == State::Fault);
    static_assert(transition(State::Fault, Event::Reset)      == State::Idle);
    // Unknown event stays in current state:
    static_assert(transition(State::Idle, Event::Stop)        == State::Idle);
    static_assert(transition(State::Configured, Event::Reset) == State::Configured);

    std::cout << "ex05_hard passed\n";
}

