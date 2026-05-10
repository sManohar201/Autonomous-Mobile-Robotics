#include <cassert>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <vector>

std::optional<int> at_checked(const std::vector<int>& v, std::size_t i) {
    if (i >= v.size()) return std::nullopt;
    return v[i];
}

std::optional<int> add_checked(int a, int b) {
    if (b > 0 && a > std::numeric_limits<int>::max() - b) return std::nullopt;
    return a + b;
}

class SafeCounter {
public:
    void inc() { std::lock_guard<std::mutex> lock{m_}; ++v_; }
    int value() const { std::lock_guard<std::mutex> lock{m_}; return v_; }
private:
    mutable std::mutex m_;
    int v_{0};
};

int main() {
    assert(!at_checked({1,2,3}, 4));
    assert(!add_checked(std::numeric_limits<int>::max(), 1));
    SafeCounter c; c.inc(); assert(c.value() == 1);
    std::cout << "ex04_hard_answer passed\n";
}

