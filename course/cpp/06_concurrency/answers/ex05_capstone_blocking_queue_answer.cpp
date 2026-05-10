#include <cassert>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <iostream>
#include <mutex>
#include <optional>
#include <thread>

template <typename T>
class BoundedBlockingQueue {
public:
    explicit BoundedBlockingQueue(std::size_t capacity)
        : capacity_{capacity}
    {
        if (capacity_ == 0) throw std::invalid_argument("capacity must be > 0");
    }

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

    std::size_t size() const {
        std::lock_guard<std::mutex> lock{mutex_};
        return queue_.size();
    }

private:
    std::size_t capacity_;
    mutable std::mutex mutex_;
    std::condition_variable not_empty_;
    std::condition_variable not_full_;
    std::deque<T> queue_;
    bool closed_{false};
};

int main() {
    BoundedBlockingQueue<int> q{2};
    std::thread producer{[&] {
        assert(q.push(1));
        assert(q.push(2));
        q.close();
    }};
    assert(q.pop().value() == 1);
    assert(q.pop().value() == 2);
    assert(!q.pop());
    producer.join();
    std::cout << "ex05_capstone_blocking_queue_answer passed\n";
}

