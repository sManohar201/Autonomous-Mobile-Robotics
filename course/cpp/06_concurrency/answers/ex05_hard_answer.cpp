#include <cassert>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

template <typename T>
class BoundedQueue {
public:
    explicit BoundedQueue(std::size_t capacity) : capacity_{capacity} {}
    bool push(T value) {
        std::unique_lock<std::mutex> lock{mutex_};
        not_full_.wait(lock, [&] { return closed_ || queue_.size() < capacity_; });
        if (closed_) return false;
        queue_.push_back(std::move(value));
        not_empty_.notify_one();
        return true;
    }
    std::optional<T> pop() {
        std::unique_lock<std::mutex> lock{mutex_};
        not_empty_.wait(lock, [&] { return closed_ || !queue_.empty(); });
        if (queue_.empty()) return std::nullopt;
        T value = std::move(queue_.front());
        queue_.pop_front();
        not_full_.notify_one();
        return value;
    }
    void close() {
        {
            std::lock_guard<std::mutex> lock{mutex_};
            closed_ = true;
        }
        not_empty_.notify_all();
        not_full_.notify_all();
    }
private:
    std::size_t capacity_;
    std::mutex mutex_;
    std::condition_variable not_empty_;
    std::condition_variable not_full_;
    std::deque<T> queue_;
    bool closed_{false};
};

int main() {
    BoundedQueue<int> q{4};
    std::thread producer{[&] { for (int i = 0; i < 10; ++i) assert(q.push(i)); q.close(); }};
    int sum = 0;
    while (auto v = q.pop()) sum += *v;
    producer.join();
    assert(sum == 45);
    std::cout << "ex05_hard_answer passed\n";
}
