#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <utility>

struct ImuSample {
    std::uint64_t timestamp_ns;
    double ax, ay, az;
};

template <typename T>
concept Timestamped = requires(const T& v) {
    { v.timestamp_ns } -> std::convertible_to<std::uint64_t>;
};

template <typename T, std::size_t Capacity>
class RingBuffer {
public:
    static_assert(Capacity > 0, "RingBuffer capacity must be greater than zero");

    void push(const T& value) {
        data_[head_] = value;
        advance_after_insert();
    }

    void push(T&& value) {
        data_[head_] = std::move(value);
        advance_after_insert();
    }

    constexpr std::size_t capacity() const { return Capacity; }
    std::size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }
    bool full() const { return size_ == Capacity; }

    T& at(std::size_t index) {
        return const_cast<T&>(static_cast<const RingBuffer&>(*this).at(index));
    }

    const T& at(std::size_t index) const {
        if (index >= size_) {
            throw std::out_of_range("ring buffer index out of range");
        }
        return data_[physical_index(index)];
    }

    const T& oldest() const {
        if (empty()) throw std::out_of_range("ring buffer is empty");
        return at(0);
    }

    const T& newest() const {
        if (empty()) throw std::out_of_range("ring buffer is empty");
        return at(size_ - 1);
    }

private:
    void advance_after_insert() {
        head_ = (head_ + 1) % Capacity;
        if (size_ < Capacity) {
            ++size_;
        }
    }

    std::size_t physical_index(std::size_t logical_index) const {
        const std::size_t oldest = full() ? head_ : 0;
        return (oldest + logical_index) % Capacity;
    }

    T data_[Capacity]{};
    std::size_t head_{0};
    std::size_t size_{0};
};

template <Timestamped T, std::size_t Capacity>
std::uint64_t latest_timestamp(const RingBuffer<T, Capacity>& buffer) {
    return buffer.newest().timestamp_ns;
}

int main() {
    RingBuffer<ImuSample, 3> buf;

    assert(buf.empty());
    buf.push(ImuSample{10, 0.0, 0.0, 9.8});
    buf.push(ImuSample{20, 0.1, 0.0, 9.7});
    buf.push(ImuSample{30, 0.2, 0.0, 9.6});
    assert(buf.full());
    assert(buf.oldest().timestamp_ns == 10);
    assert(buf.newest().timestamp_ns == 30);

    buf.push(ImuSample{40, 0.3, 0.0, 9.5});
    assert(buf.size() == 3);
    assert(buf.oldest().timestamp_ns == 20);
    assert(buf.at(0).timestamp_ns == 20);
    assert(buf.at(2).timestamp_ns == 40);
    assert(latest_timestamp(buf) == 40);

    std::cout << "ex05_capstone_typed_ring_buffer_answer passed\n";
}

