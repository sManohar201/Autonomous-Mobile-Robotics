#include <cassert>
#include <map>
#include <optional>
#include <sstream>
#include <iostream>
#include <string>
#include <variant>

using ConfigValue = std::variant<int, double, bool, std::string>;
using ConfigMap = std::map<std::string, ConfigValue>;

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
    auto rate = get<double>(cfg, "rate_hz");
    auto frame = get<std::string>(cfg, "frame_id");
    if (!rate || *rate <= 0.0) { error = "rate_hz must be positive double"; return std::nullopt; }
    if (!frame || frame->empty()) { error = "frame_id must be non-empty string"; return std::nullopt; }
    return ControllerConfig{*rate, *frame, get<bool>(cfg, "debug").value_or(false)};
}

int main() {
    ConfigMap cfg{{"rate_hz", 50.0}, {"frame_id", std::string{"base_link"}}, {"debug", true}};
    std::string error;
    auto parsed = parse(cfg, error);
    assert(parsed);
    assert(parsed->debug);
    assert(stringify(ConfigValue{42}) == "42");
    std::cout << "ex03_hard_answer passed\n";
}

