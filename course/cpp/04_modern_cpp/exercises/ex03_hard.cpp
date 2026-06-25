// Exercise 03 (Hard) - Variant Config Parser with Validation Errors
//
// Tasks:
//   1. Define ConfigValue = variant<int, double, bool, string>.
//   2. Return optional<ControllerConfig> for valid configs.
//   3. Return validation error strings for invalid configs.
//   4. Use std::visit to stringify ConfigValue.

#include <cassert>
#include <map>
#include <optional>
#include <sstream>
#include <iostream>
#include <string>
#include <variant>

using ConfigValue = std::variant<int, double, bool, std::string>;
using ConfigMap   = std::map<std::string, ConfigValue>;

struct ControllerConfig {
    double rate_hz;
    std::string frame_id;
    bool debug;
};

std::string stringify(const ConfigValue& value) {
    return std::visit([](const auto& v) {
        std::ostringstream os;
        os << v;
        return os.str();
    }, value);
}

template <typename T>
std::optional<T> get(const ConfigMap& cfg, const std::string& key) {
    auto it = cfg.find(key);
    if (it == cfg.end()) return std::nullopt;
    if (auto ptr = std::get_if<T>(&it->second)) return *ptr;
    return std::nullopt;
}

std::optional<ControllerConfig> parse(const ConfigMap& cfg, std::string& error) {
    auto rate  = get<double>(cfg, "rate_hz");
    auto frame = get<std::string>(cfg, "frame_id");
    if (!rate || *rate <= 0.0)    { error = "rate_hz must be positive double"; return std::nullopt; }
    if (!frame || frame->empty()) { error = "frame_id must be non-empty string"; return std::nullopt; }
    return ControllerConfig{*rate, *frame, get<bool>(cfg, "debug").value_or(false)};
}

int main() {
    ConfigMap cfg{{"rate_hz", 50.0}, {"frame_id", std::string{"base_link"}}, {"debug", true}};
    std::string error;
    auto parsed = parse(cfg, error);
    assert(parsed);
    assert(parsed->debug);
    assert(error.empty());

    ConfigMap bad{{"rate_hz", -1.0}, {"frame_id", std::string{"base_link"}}};
    auto bad_result = parse(bad, error);
    assert(!bad_result);
    assert(!error.empty());

    assert(stringify(ConfigValue{42}) == "42");
    assert(stringify(ConfigValue{3.14}) == "3.14");
    assert(stringify(ConfigValue{true}) == "1");

    std::cout << "ex03_hard passed\n";
}

