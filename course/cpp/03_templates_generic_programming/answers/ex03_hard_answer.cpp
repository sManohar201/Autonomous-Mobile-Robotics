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

template <typename T>
concept TimedFrameMessage = HasTimestamp<T> && HasFrameId<T>;

struct ImuMsg { std::uint64_t timestamp_ns; std::string frame_id; };
struct ClockMsg { std::uint64_t timestamp_ns; };
struct TextMsg { std::string text; };

template <HasTimestamp Msg>
std::uint64_t latency_ns(std::uint64_t now, const Msg& msg) {
    return now - msg.timestamp_ns;
}

template <typename Msg>
std::string describe(const Msg& msg) {
    std::ostringstream os;
    if constexpr (HasTimestamp<Msg>) os << "t=" << msg.timestamp_ns;
    else os << "t=<none>";
    if constexpr (HasFrameId<Msg>) os << " frame=" << msg.frame_id;
    else os << " frame=<none>";
    return os.str();
}

int main() {
    static_assert(TimedFrameMessage<ImuMsg>);
    static_assert(HasTimestamp<ClockMsg>);
    static_assert(!HasTimestamp<TextMsg>);
    ImuMsg imu{100, "imu_link"};
    assert(latency_ns(150, imu) == 50);
    assert(describe(imu) == "t=100 frame=imu_link");
    assert(describe(TextMsg{"ok"}) == "t=<none> frame=<none>");
    std::cout << "ex03_hard_answer passed\n";
}

