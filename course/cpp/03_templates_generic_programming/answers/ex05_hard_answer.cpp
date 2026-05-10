#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <iostream>

template <typename T>
concept Timestamped = requires(const T& v) {
    { v.timestamp_ns } -> std::convertible_to<std::uint64_t>;
};

template <typename T, std::size_t Capacity>
class RingBuffer {
public:
    static_assert(Capacity > 0);

    void push(const T& value) {
        data_[head_] = value;
        head_ = (head_ + 1) % Capacity;
        if (size_ < Capacity) ++size_;
    }

    std::size_t size() const { return size_; }
    bool full() const { return size_ == Capacity; }

    const T& at(std::size_t i) const {
        return data_[(oldest_index() + i) % Capacity];
    }

    class const_iterator {
    public:
        const_iterator(const RingBuffer* owner, std::size_t index) : owner_{owner}, index_{index} {}
        const T& operator*() const { return owner_->at(index_); }
        const_iterator& operator++() { ++index_; return *this; }
        bool operator!=(const const_iterator& other) const { return index_ != other.index_; }
    private:
        const RingBuffer* owner_;
        std::size_t index_;
    };

    const_iterator begin() const { return {this, 0}; }
    const_iterator end() const { return {this, size_}; }

private:
    std::size_t oldest_index() const { return full() ? head_ : 0; }
    T data_[Capacity]{};
    std::size_t head_{0};
    std::size_t size_{0};
};

template <Timestamped T, std::size_t N>
std::uint64_t latest_timestamp(const RingBuffer<T, N>& buffer) {
    return buffer.at(buffer.size() - 1).timestamp_ns;
}

struct Sample { std::uint64_t timestamp_ns; int value; };

int main() {
    RingBuffer<Sample, 3> b;
    b.push({10, 1}); b.push({20, 2}); b.push({30, 3}); b.push({40, 4});
    assert(b.full());
    assert(b.at(0).timestamp_ns == 20);
    assert(latest_timestamp(b) == 40);
    int sum = 0;
    for (const auto& s : b) sum += s.value;
    assert(sum == 9);
    std::cout << "ex05_hard_answer passed\n";
}
