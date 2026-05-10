#include <cassert>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <optional>
#include <queue>

class IntQueue {
public:
    void push(int value) {
        {
            std::lock_guard<std::mutex> lock{mutex_};
            queue_.push(value);
        }
        cv_.notify_one();
    }

    std::optional<int> pop() {
        std::unique_lock<std::mutex> lock{mutex_};
        cv_.wait(lock, [&] { return closed_ || !queue_.empty(); });
        if (queue_.empty()) return std::nullopt;
        int value = queue_.front();
        queue_.pop();
        return value;
    }

    void close() {
        {
            std::lock_guard<std::mutex> lock{mutex_};
            closed_ = true;
        }
        cv_.notify_all();
    }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<int> queue_;
    bool closed_{false};
};

int main() {
    IntQueue q;
    q.push(7);
    assert(q.pop().value() == 7);
    q.close();
    assert(!q.pop());
    std::cout << "ex03_condition_variable_answer passed\n";
}

