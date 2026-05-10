#include <cassert>
#include <iostream>
#include <map>
#include <string>
#include <vector>

struct Payload {
    std::vector<std::byte> bytes;
    double rate_hz;
};

std::size_t total_payload_bytes(const std::map<std::string, Payload>& payloads) {
    std::size_t total = 0;
    for (const auto& [name, payload] : payloads) {
        (void)name;
        total += payload.bytes.size();
    }
    return total;
}

bool mutate_rate(std::map<std::string, Payload>& payloads, const std::string& key, double rate) {
    auto it = payloads.find(key);
    if (it == payloads.end()) return false;
    auto& [name, payload] = *it;
    (void)name;
    payload.rate_hz = rate;
    return true;
}

int main() {
    std::map<std::string, Payload> payloads;
    payloads["imu"] = Payload{std::vector<std::byte>(12), 100.0};
    payloads["gps"] = Payload{std::vector<std::byte>(4), 10.0};
    assert(total_payload_bytes(payloads) == 16);
    assert(mutate_rate(payloads, "gps", 5.0));
    assert(payloads["gps"].rate_hz == 5.0);
    std::cout << "ex01_hard_answer passed\n";
}

