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

template <typename T>
std::optional<T> get_as(const ConfigMap& cfg, const std::string& key) {
    const auto it = cfg.find(key);
    if (it == cfg.end()) {
        return std::nullopt;
    }

    if (const auto* value = std::get_if<T>(&it->second)) {
        return *value;
    }

    return std::nullopt;
}

std::optional<ControllerConfig> parse_controller_config(const ConfigMap& cfg) {
    const auto rate_hz = get_as<double>(cfg, "rate_hz");
    const auto frame_id = get_as<std::string>(cfg, "frame_id");

    if (!rate_hz || *rate_hz <= 0.0) {
        return std::nullopt;
    }
    if (!frame_id || frame_id->empty()) {
        return std::nullopt;
    }

    return ControllerConfig{
        *rate_hz,
        *frame_id,
        get_as<bool>(cfg, "publish_debug").value_or(false),
    };
}

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

constexpr State transition(State state, Event event) {
    if (event == Event::Error) {
        return State::Fault;
    }

    switch (state) {
    case State::Idle:
        return event == Event::Configure ? State::Configured : state;
    case State::Configured:
        return event == Event::Start ? State::Running : state;
    case State::Running:
        return event == Event::Stop ? State::Configured : state;
    case State::Fault:
        return event == Event::Reset ? State::Idle : state;
    }

    return State::Fault;
}

int main() {
    ConfigMap cfg{
        {"rate_hz", 50.0},
        {"frame_id", std::string{"base_link"}},
        {"publish_debug", true},
    };

    auto parsed = parse_controller_config(cfg);
    assert(parsed);
    assert(parsed->rate_hz == 50.0);
    assert(parsed->frame_id == "base_link");
    assert(parsed->publish_debug);

    ConfigMap invalid{
        {"rate_hz", -1.0},
        {"frame_id", std::string{"base_link"}},
    };
    assert(!parse_controller_config(invalid));

    static_assert(transition(State::Idle, Event::Configure) == State::Configured);
    static_assert(transition(State::Configured, Event::Start) == State::Running);
    static_assert(transition(State::Running, Event::Stop) == State::Configured);
    static_assert(transition(State::Running, Event::Error) == State::Fault);
    static_assert(transition(State::Fault, Event::Reset) == State::Idle);

    std::cout << "ex05_capstone_config_state_machine_answer passed\n";
}

