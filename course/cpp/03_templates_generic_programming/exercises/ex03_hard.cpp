// Exercise 03 (Hard) - Detection Idiom and Concepts for Sensor Messages
//
// Context:
//   Generic loggers need to accept any message with timestamp/frame fields,
//   while producing clear compiler errors for invalid message types.
//
// Tasks:
//   1. Define concepts HasTimestamp, HasFrameId, and TimedFrameMessage.
//   2. Implement latency_ns(now, msg), constrained to HasTimestamp.
//   3. Implement describe(msg), using if constexpr for optional frame_id.
//   4. Add static_assert tests for valid and invalid message structs.

#include <cassert>
#include <concepts>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>

template <typename T>
concept HasTimestamp = requires(const T& msg) {
    { msg.timestamp_ns } -> std::convertible_to<std::uint64_t>;
};

template <typename T>
concept HasFrameId = requires(const T& msg) {
    { msg.frame_id } -> std::convertible_to<std::string>;
};

// Conjunction concept: must satisfy both.
template <typename T>
concept TimedFrameMessage = HasTimestamp<T> && HasFrameId<T>;

// ── Message types ─────────────────────────────────────────────────────────────

struct ImuMsg   { std::uint64_t timestamp_ns; std::string frame_id; };
struct ClockMsg { std::uint64_t timestamp_ns; };
struct TextMsg  { std::string text; };

// ── latency_ns — constrained to HasTimestamp ──────────────────────────────────

template <HasTimestamp Msg>
std::uint64_t latency_ns(std::uint64_t now, const Msg& msg) {
    return now - msg.timestamp_ns;
}

// ── describe — uses if constexpr for optional fields ─────────────────────────

template <typename Msg>
std::string describe(const Msg& msg) {
    std::ostringstream os;
    if constexpr (HasTimestamp<Msg>) {
        os << "t=" << msg.timestamp_ns;
    } else {
        os << "t=<none>";
    }
    if constexpr (HasFrameId<Msg>) {
        os << " frame=" << msg.frame_id;
    } else {
        os << " frame=<none>";
    }
    return os.str();
}

int main() {
    static_assert(TimedFrameMessage<ImuMsg>);
    static_assert(HasTimestamp<ClockMsg>);
    static_assert(!HasTimestamp<TextMsg>);
    static_assert(!HasFrameId<ClockMsg>);

    ImuMsg imu{100, "imu_link"};
    assert(latency_ns(150, imu) == 50);
    assert(describe(imu) == "t=100 frame=imu_link");
    assert(describe(ClockMsg{200}) == "t=200 frame=<none>");
    assert(describe(TextMsg{"ok"}) == "t=<none> frame=<none>");

    std::cout << "ex03_hard passed\n";
}

