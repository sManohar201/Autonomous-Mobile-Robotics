#include <cassert>
#include <iostream>
#include <optional>
#include <string_view>

std::string_view trim(std::string_view v) {
    while (!v.empty() && v.front() == ' ') v.remove_prefix(1);
    while (!v.empty() && v.back() == ' ') v.remove_suffix(1);
    return v;
}

struct KeyValue { std::string_view key; std::string_view value; };

std::optional<KeyValue> parse_line(std::string_view line) {
    auto pos = line.find('=');
    if (pos == std::string_view::npos) return std::nullopt;
    return KeyValue{trim(line.substr(0, pos)), trim(line.substr(pos + 1))};
}

int main() {
    auto kv = parse_line(" rate_hz = 50 ");
    assert(kv);
    assert(kv->key == "rate_hz");
    assert(kv->value == "50");
    assert(!parse_line("invalid"));
    std::cout << "ex04_hard_answer passed\n";
}

