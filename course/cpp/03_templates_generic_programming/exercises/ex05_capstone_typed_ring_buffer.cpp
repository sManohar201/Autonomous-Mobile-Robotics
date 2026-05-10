// Exercise 05 Capstone - Generic Typed Ring Buffer
//
// Goal:
//   Build a fixed-capacity ring buffer with compile-time constraints and
//   timestamp-aware helpers.

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

    // TODO 1: push(const T& value).
    // Insert at the logical back. If full, overwrite the oldest element.

    // TODO 2: push(T&& value).
    // Same behavior, but move into storage.

    // TODO 3: size(), capacity(), empty(), full().

    // TODO 4: at(index), const and non-const.
    // Logical index 0 is the oldest element.
    // Throw std::out_of_range if index >= size().

    // TODO 5: newest() and oldest(), const.
    // Throw std::out_of_range if empty.

private:
    // TODO: choose storage and indices.
};

// TODO 6: latest_timestamp(buffer), constrained to Timestamped T.
// Returns newest().timestamp_ns.

int main() {
    RingBuffer<ImuSample, 3> buf;

    // TODO: make these pass.
    // assert(buf.empty());
    // buf.push(ImuSample{10, 0.0, 0.0, 9.8});
    // buf.push(ImuSample{20, 0.1, 0.0, 9.7});
    // buf.push(ImuSample{30, 0.2, 0.0, 9.6});
    // assert(buf.full());
    // assert(buf.oldest().timestamp_ns == 10);
    // assert(buf.newest().timestamp_ns == 30);
    //
    // buf.push(ImuSample{40, 0.3, 0.0, 9.5});
    // assert(buf.size() == 3);
    // assert(buf.oldest().timestamp_ns == 20);
    // assert(buf.at(0).timestamp_ns == 20);
    // assert(buf.at(2).timestamp_ns == 40);
    // assert(latest_timestamp(buf) == 40);

    std::cout << "ex05_capstone_typed_ring_buffer passed\n";
}

