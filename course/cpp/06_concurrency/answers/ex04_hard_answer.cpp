#include <atomic>
#include <cassert>
#include <iostream>
#include <mutex>
#include <vector>

class SafeSamples {
public:
    void push(int value) {
        std::lock_guard<std::mutex> lock{mutex_};
        samples_.push_back(value);
    }
    std::size_t size() const {
        std::lock_guard<std::mutex> lock{mutex_};
        return samples_.size();
    }
private:
    mutable std::mutex mutex_;
    std::vector<int> samples_;
};

int main() {
    std::atomic<int> dropped{0};
    ++dropped;
    assert(dropped.load() == 1);
    SafeSamples samples;
    samples.push(5);
    assert(samples.size() == 1);
    std::cout << "ex04_hard_answer passed\n";
}

