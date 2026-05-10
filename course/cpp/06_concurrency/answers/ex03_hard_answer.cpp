#include <cassert>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <mutex>
#include <optional>

template <typename T>
class BlockingQueue {
public:
    void push(T value) {
        {
            std::lock_guard<std::mutex> lock{mutex_};
            queue_.push_back(std::move(value));
        }
        cv_.notify_one();
    }
    std::optional<T> pop() {
        std::unique_lock<std::mutex> lock{mutex_};
        cv_.wait(lock, [&] { return closed_ || !queue_.empty(); });
        if (queue_.empty()) return std::nullopt;
        T value = std::move(queue_.front());
        queue_.pop_front();
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
    std::deque<T> queue_;
    bool closed_{false};
};

int main() {
    BlockingQueue<int> q;
    q.push(1);
    assert(q.pop().value() == 1);
    q.close();
    assert(!q.pop());
    std::cout << "ex03_hard_answer passed\n";
}

