// ANSWER — Exercise 02 (Hard): Integer Promotion, Signed/Unsigned Traps,
//                              Endianness, and Safe Sensor Protocol Parsing
// ─────────────────────────────────────────────────────────────────────────────

// ── Q1 ────────────────────────────────────────────────────────────────────────
// `uint8_t a = 200; uint8_t b = 100; auto c = a + b;`
//
// Type of c: int (NOT uint8_t)
// Value: 300
//
// Why? Integer promotion: before any arithmetic, operands smaller than int
// are converted to int. Both a and b are promoted to int before the addition.
// The addition 200 + 100 = 300 happens entirely in int space (32-bit), so
// there is no 8-bit wraparound. The result type is int and the value is 300.
//
// "Why doesn't it wrap?" — because it IS NOT 8-bit addition. The uint8_t
// values are promoted to int before the + operator is applied. Wrapping would
// only occur if the arithmetic were performed in uint8_t, which it is not.

// ── Q2 ────────────────────────────────────────────────────────────────────────
// `int neg = -1; uint32_t pos = 0; bool result = (neg < pos);`
// result = false (neg is NOT less than pos)
//
// Step-by-step conversion:
//   1. Operator < has two operands: int (-1) and uint32_t (0).
//   2. Usual arithmetic conversions: when int and unsigned int are mixed, the
//      SIGNED int is converted to UNSIGNED.
//   3. -1 converted to uint32_t: -1 wraps to 0xFFFFFFFF = 4294967295.
//   4. Comparison: 4294967295 < 0  → false.
// Output: "not less"
// This is one of the most common C/C++ bugs: a signed check against a
// container size() (which returns size_t, an unsigned type).

// ── Q3 ────────────────────────────────────────────────────────────────────────
// Endianness: the order in which bytes of a multi-byte value are stored in
// memory, starting at the lowest address.
//   - Little-endian: LSB (least significant byte) at the lowest address.
//     Example: 0x00001234 stored as [0x34, 0x12, 0x00, 0x00] in memory.
//   - Big-endian (network byte order): MSB at the lowest address.
//     Example: 0x00001234 stored as [0x00, 0x00, 0x12, 0x34].
//
// x86-64 is LITTLE-ENDIAN.
//
// To reconstruct a 32-bit big-endian timestamp on a little-endian machine:
//   uint32_t ts = (uint32_t(buf[0]) << 24) | (uint32_t(buf[1]) << 16)
//               | (uint32_t(buf[2]) <<  8) |  uint32_t(buf[3]);
// Shifting and OR-ing is always correct regardless of host endianness, because
// we are constructing the value from individual bytes in a portable way.

// ── Q4 ────────────────────────────────────────────────────────────────────────
// `int32_t val = *reinterpret_cast<int32_t*>(byte_buf);` — WHY UB?
//
// The C++ strict aliasing rule: the compiler is allowed to assume that two
// pointers of different types do not point to the same memory (with exceptions
// for char*/uint8_t*). Accessing memory through a type other than its actual
// type is undefined behavior.
//
// Here: byte_buf is uint8_t[]. Its actual type is array of uint8_t. Reading
// through an int32_t* violates strict aliasing. The compiler may reorder or
// eliminate such reads, leading to incorrect behavior in optimized builds.
//
// Safe C++17 alternatives:
//   1. std::memcpy(&val, buf, 4);   — well-defined: byte copy, no alias issue
//   2. std::bit_cast<int32_t>(arr); — C++20, type-punning via value reinterpretation
//   3. Manual bit-shift reconstruction (used in this exercise) — always portable

// ── Q5 ────────────────────────────────────────────────────────────────────────
// `for (uint8_t i = 0; i < 256; ++i)` — INFINITE LOOP.
//
// Trace:
//   - i is uint8_t, range [0, 255].
//   - The constant 256 is an int. For the comparison, i is promoted to int.
//   - Loop body executes for i = 0, 1, ..., 255.
//   - At i = 255: ++i is performed. The result of 255+1=256 is int, but then
//     it is stored back into the uint8_t, which wraps to 0 (unsigned overflow
//     is well-defined, wraps modulo 256).
//   - Now i = 0 again. The condition 0 < 256 is true → loop repeats forever.
//
// Fix: use `int i` or `for (int i = 0; i < 256; ++i)`.

// ── Q6 ────────────────────────────────────────────────────────────────────────
// (a) Can a constexpr function be called at runtime?
//     YES. A constexpr function CAN be called at runtime with runtime values.
//     The constexpr keyword means "may be evaluated at compile time"; it is NOT
//     a promise that it will always be. When all arguments are compile-time
//     constants and the result is used in a constant expression context
//     (e.g., array size, template argument, static_assert), it IS evaluated at
//     compile time. Otherwise it runs like a normal function.
//
// (b) Can it contain if/for in C++17?
//     YES. C++14 relaxed constexpr to allow if, for, while, local variables,
//     and more. C++17 further relaxed it. Only features not known at compile
//     time (virtual, exceptions that propagate, non-constexpr calls) are
//     disallowed.
//
// (c) constexpr vs const:
//     const: "this value will not be modified after initialization."
//            Initialization can happen at RUNTIME. Example: `const int n = argc;`
//     constexpr: "this value IS KNOWN at compile time."
//            Initialization MUST be a constant expression.
//            constexpr implies const for variables.

// ─────────────────────────────────────────────────────────────────────────────

#include <iostream>
#include <iomanip>
#include <cstdint>
#include <cstring>
#include <stdexcept>

void part_a() {
    std::cout << std::fixed << std::setprecision(5);

    // Snippet 1 — integer promotion
    // PREDICT: 300 (not 44). uint8_t promoted to int before addition.
    {
        uint8_t a = 200, b = 100;
        auto c = a + b;
        std::cout << "snippet1: " << c << "\n";
    }

    // Snippet 2 — signed/unsigned comparison
    // PREDICT: not less. -1 converts to uint32_t → 4294967295 > 0.
    // The -Wsign-compare warning here is INTENTIONAL — it's the very bug we're
    // demonstrating. In production code you must fix this; here we leave it.
    {
        int neg = -1;
        uint32_t pos = 0;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
        std::cout << "snippet2: " << (neg < pos ? "less" : "not less") << "\n";
#pragma GCC diagnostic pop
    }

    // Snippet 3 — uint8_t wraparound
    // PREDICT: 0. 255 + 1 in int is 256; stored to uint8_t wraps to 0.
    {
        uint8_t count = 255;
        count += 1;
        std::cout << "snippet3: " << static_cast<int>(count) << "\n";
    }

    // Snippet 4 — int16_t divided by floating-point literal
    // PREDICT: -0.97656. -32000 / 32768.0 = -0.976562...
    // int16_t is converted to double before the division (no integer division).
    {
        int16_t raw = -32000;
        double scaled = raw / 32768.0;
        std::cout << "snippet4: " << scaled << "\n";
    }

    // Snippet 5 — two comparisons, one with explicit cast
    // snippet5a: idx < static_cast<int>(limit) → -1 < 3 → true → "A"
    // snippet5b: idx < limit → int vs unsigned int → -1 wraps → huge > 3 → "D"
    {
        unsigned int limit = 3;
        int idx = -1;
        std::cout << "snippet5a: " << (idx < static_cast<int>(limit) ? "A" : "B") << "\n";
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
        std::cout << "snippet5b: " << (idx < limit                   ? "C" : "D") << "\n";
#pragma GCC diagnostic pop
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Part B — constexpr sensor utility functions
// ─────────────────────────────────────────────────────────────────────────────

// B1. Raw ADC to m/s².
// 16-bit signed ADC, full-scale ±full_scale_g g.
// Raw 32768 would be full scale (but int16_t max is 32767, so 32768.0 is the
// theoretical full-scale denominator per the sensor convention).
constexpr double lsb_to_ms2(int16_t raw, double full_scale_g) {
    return (raw / 32768.0) * full_scale_g * 9.80665;
}
// Verified at compile time:
static_assert(lsb_to_ms2(0, 2.0) == 0.0, "lsb_to_ms2(0, 2.0) must be 0");
// lsb_to_ms2(16384, 2.0) = (16384/32768) * 2 * 9.80665 = 0.5 * 2 * 9.80665 = 9.80665

// B2. Milliseconds to microseconds, overflow-safe.
// A naive multiply overflows for ms > ~4294967 (just under 2^32 / 1000).
// Saturate to UINT32_MAX rather than wrapping silently.
constexpr uint32_t ms_to_us(uint32_t ms) {
    return ms > (UINT32_MAX / 1000u) ? UINT32_MAX : ms * 1000u;
}
static_assert(ms_to_us(1000u) == 1000000u, "ms_to_us(1000) must be 1000000");
// ms_to_us(4294968): 4294968 > 4294967 (UINT32_MAX/1000) → returns UINT32_MAX

// B3. IMU start-byte validation via high nibble.
// High nibble = b >> 4. Must equal 0xA (decimal 10).
constexpr bool is_valid_imu_header(uint8_t b) {
    return (b >> 4) == 0xAu;
}
// 0xA5 >> 4 = 0xA → true
// 0xB0 >> 4 = 0xB → false

// ─────────────────────────────────────────────────────────────────────────────
// Part C — Safe IMU binary packet parser
// ─────────────────────────────────────────────────────────────────────────────

struct ImuPacket {
    uint32_t timestamp_ms;
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
};

// WHY is reinterpret_cast<int32_t*>(buf) undefined behavior here?
//
// The strict aliasing rule (C++17 §6.7): an object of type T may only be
// accessed through a pointer (or reference or glvalue) of a compatible type.
// Compatible types are: T itself, const/volatile T, a base class of T, or
// (as a special exception) char, unsigned char, or std::byte.
//
// buf points to uint8_t (unsigned char). Dereferencing a reinterpret_cast to
// int32_t* treats the same bytes as a different, incompatible type. The
// compiler is allowed to assume these two pointers can never alias, so it may
// reorder, cache, or eliminate accesses. In optimized builds (-O2) this
// regularly produces WRONG results.
//
// SAFE alternative: use std::memcpy (explicitly defined behavior for type-punning)
// or reconstruct from bytes using shifts+OR (used here — unambiguously portable).

ImuPacket parse_packet(const uint8_t* buf, int len) {
    if (len < 16) {
        throw std::runtime_error("invalid packet");
    }

    ImuPacket pkt{};

    // Timestamp — big-endian reconstruction (shift and OR, no memcpy/cast)
    pkt.timestamp_ms = (uint32_t(buf[0]) << 24)
                     | (uint32_t(buf[1]) << 16)
                     | (uint32_t(buf[2]) <<  8)
                     |  uint32_t(buf[3]);

    // int16_t values — little-endian reconstruction
    // buf[i] = low byte, buf[i+1] = high byte
    // Cast through uint16_t first to ensure unsigned widening before the shift
    auto read_le16 = [](const uint8_t* p) -> int16_t {
        return static_cast<int16_t>((uint16_t(p[1]) << 8) | uint16_t(p[0]));
    };

    pkt.ax = read_le16(buf + 4);
    pkt.ay = read_le16(buf + 6);
    pkt.az = read_le16(buf + 8);
    pkt.gx = read_le16(buf + 10);
    pkt.gy = read_le16(buf + 12);
    pkt.gz = read_le16(buf + 14);

    return pkt;
}

// ─────────────────────────────────────────────────────────────────────────────
// EXPECTED OUTPUT:
//   snippet1: 300
//   snippet2: not less
//   snippet3: 0
//   snippet4: -0.97656
//   snippet5a: A
//   snippet5b: D
//
//   lsb_to_ms2(16384, 2.0) = 9.80665 m/s2
//   ms_to_us(1000) = 1000000
//   ms_to_us(4294968) = 4294967295
//   is_valid_imu_header(0xA5) = 1
//   is_valid_imu_header(0xB0) = 0
//   parsed: timestamp=4000ms ax=-448 ay=0 az=2048 gx=1000 gy=0 gz=0
// ─────────────────────────────────────────────────────────────────────────────

int main() {
    part_a();

    std::cout << "\n";

    std::cout << std::fixed << std::setprecision(5);
    std::cout << "lsb_to_ms2(16384, 2.0) = "
              << lsb_to_ms2(16384, 2.0) << " m/s2\n";
    std::cout << "ms_to_us(1000) = "    << ms_to_us(1000u)    << "\n";
    std::cout << "ms_to_us(4294968) = " << ms_to_us(4294968u) << "\n";
    std::cout << "is_valid_imu_header(0xA5) = "
              << static_cast<int>(is_valid_imu_header(0xA5)) << "\n";
    std::cout << "is_valid_imu_header(0xB0) = "
              << static_cast<int>(is_valid_imu_header(0xB0)) << "\n";

    uint8_t test_packet[] = {
        0x00, 0x00, 0x0F, 0xA0,  // timestamp big-endian: 0x00000FA0 = 4000
        0x40, 0xFE,              // ax: 0xFE40 LE → signed -448
        0x00, 0x00,              // ay: 0
        0x00, 0x08,              // az: 0x0800 LE = 2048
        0xE8, 0x03,              // gx: 0x03E8 LE = 1000
        0x00, 0x00,              // gy: 0
        0x00, 0x00               // gz: 0
    };

    ImuPacket pkt = parse_packet(test_packet, 16);
    std::cout << "parsed: timestamp=" << pkt.timestamp_ms << "ms"
              << " ax=" << pkt.ax
              << " ay=" << pkt.ay
              << " az=" << pkt.az
              << " gx=" << pkt.gx
              << " gy=" << pkt.gy
              << " gz=" << pkt.gz
              << "\n";

    return 0;
}
