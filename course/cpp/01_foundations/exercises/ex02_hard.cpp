// Exercise 02 (Hard) — Integer Promotion, Signed/Unsigned Traps, Endianness,
//                      and Safe Sensor Protocol Parsing
// ─────────────────────────────────────────────────────────────────────────────
//
// CONTEXT:
//   IMU/GPS/LiDAR sensors in robotics and autonomous vehicles send binary
//   packets over UART, SPI, and CAN bus. Production systems parse these
//   packets at 1 kHz or more. Parsing incorrectly causes silent data
//   corruption that is orders of magnitude harder to find than an outright
//   crash.
//
// ─────────────────────────────────────────────────────────────────────────────
// PRE-CODING RESEARCH QUESTIONS — Answer in the spaces provided.
// ─────────────────────────────────────────────────────────────────────────────

// Q1. Integer promotion:
//     `uint8_t a = 200; uint8_t b = 100; auto c = a + b;`
//     Type of c: int (both operands are promoted to int before addition).
//     Value: 300 — no wrapping because int is wide enough (32-bit holds 300).
//     Would be 44 only if the result were stored back into uint8_t.

// Q2. Usual arithmetic conversions (signed/unsigned mixing):
//     `int neg = -1; uint32_t pos = 0; bool result = (neg < pos);`
//     Step 1: int and uint32_t are mixed. uint32_t is wider (same width on
//             64-bit if both are 32-bit) OR unsigned wins the conversion.
//     Step 2: neg (-1) is converted to uint32_t → 0xFFFFFFFF = 4294967295.
//     Step 3: 4294967295 < 0 → false.
//     result = false. The signed -1 wraps to a huge unsigned value.

// Q3. Endianness:
//     Endianness: byte order within a multi-byte value.
//       Big-endian: MSB at lowest address (network byte order).
//       Little-endian: LSB at lowest address (x86-64, ARM default, most desktop CPUs).
//     x86-64 is LITTLE-endian.
//     To reconstruct a big-endian 32-bit timestamp on little-endian:
//       ts = (uint32_t(buf[0]) << 24) | (uint32_t(buf[1]) << 16)
//          | (uint32_t(buf[2]) <<  8) |  uint32_t(buf[3]);

// Q4. Strict aliasing rule:
//     `*reinterpret_cast<int32_t*>(byte_buf)` is UB because:
//     The compiler assumes that pointers to different types never alias.
//     Reading through an int32_t* a buffer that actually holds uint8_t data
//     violates this assumption — the compiler may reorder or eliminate the read.
//     Safe C++17 alternative: std::memcpy into an int32_t local variable.
//     std::bit_cast<int32_t>(some_array) is clean but requires C++20.

// Q5. Loop trap: `for (uint8_t i = 0; i < 256; ++i)`
//     After i = 255, ++i promotes 255 to int, adds 1 → 256, then truncates back
//     to uint8_t → 0. The loop condition 0 < 256 is true again → infinite loop.

// Q6. constexpr vs const:
//     (a) Yes, a constexpr function can be called at runtime (if called with
//         non-constant arguments, it runs as a regular function).
//     (b) Yes (C++14+), constexpr functions may contain if-statements and loops.
//     (c) const means "this value does not change after initialisation" — evaluated
//         at runtime. constexpr means "evaluate at compile time if possible" — the
//         compiler must be able to compute it with constant arguments.

// ─────────────────────────────────────────────────────────────────────────────
// PART A — Predict the output of each snippet BEFORE compiling.
// ─────────────────────────────────────────────────────────────────────────────

#include <iostream>
#include <iomanip>
#include <cstdint>
#include <cstring>
#include <stdexcept>

void part_a() {
    std::cout << std::fixed << std::setprecision(5);

    // Snippet 1 — integer promotion on uint8_t addition
    // PREDICT: 300 (both promoted to int; 200+100=300, no 8-bit truncation)
    {
        uint8_t a = 200, b = 100;
        auto c = a + b;
        std::cout << "snippet1: " << c << "\n";
    }

    // Snippet 2 — signed/unsigned comparison trap
    // PREDICT: "not less" (-1 converted to uint32_t → huge positive number)
    {
        int neg = -1;
        uint32_t pos = 0;
        std::cout << "snippet2: " << (neg < pos ? "less" : "not less") << "\n";
    }

    // Snippet 3 — uint8_t overflow
    // PREDICT: 0 (255 + 1 wraps modulo 256 back to 0)
    {
        uint8_t count = 255;
        count += 1;
        std::cout << "snippet3: " << static_cast<int>(count) << "\n";
    }

    // Snippet 4 — int16_t to double division (note: integer / floating-point)
    // PREDICT: -0.97656 (-32000 / 32768.0 ≈ -0.97656)
    {
        int16_t raw = -32000;
        double scaled = raw / 32768.0;
        std::cout << "snippet4: " << scaled << "\n";
    }

    // Snippet 5 — two comparisons: one with cast, one without
    // PREDICT snippet5a (with cast): "A" (-1 < 3 as signed comparison → true)
    // PREDICT snippet5b (without cast): "D" (-1 converted to unsigned → huge, not < 3)
    {
        unsigned int limit = 3;
        int idx = -1;
        std::cout << "snippet5a: " << (idx < static_cast<int>(limit) ? "A" : "B") << "\n";
        std::cout << "snippet5b: " << (idx < limit                   ? "C" : "D") << "\n";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// PART B — constexpr sensor utility functions
// ─────────────────────────────────────────────────────────────────────────────

// B1. Convert raw ADC value to acceleration in m/s²
constexpr double lsb_to_ms2(int16_t raw, double full_scale_g) {
    return (raw / 32768.0) * full_scale_g * 9.80665;
}
static_assert(lsb_to_ms2(0, 2.0) == 0.0, "lsb_to_ms2(0, 2.0) should be 0");

// B2. Convert milliseconds to microseconds, saturating on overflow.
constexpr uint32_t ms_to_us(uint32_t ms) {
    return ms > (UINT32_MAX / 1000u) ? UINT32_MAX : ms * 1000u;
}
static_assert(ms_to_us(1000u) == 1000000u, "ms_to_us(1000) should be 1000000");

// B3. Validate an IMU start byte: the high nibble must equal 0xA.
constexpr bool is_valid_imu_header(uint8_t b) {
    return (b >> 4) == 0xA;
}

// ─────────────────────────────────────────────────────────────────────────────
// PART C — Safe IMU binary packet parser
// ─────────────────────────────────────────────────────────────────────────────

struct ImuPacket {
    uint32_t timestamp_ms;
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
};

// WHY reinterpret_cast<int32_t*>(buf) is UB here:
// The buffer holds uint8_t data. Accessing it through an int32_t* violates the
// strict aliasing rule: the compiler assumes int32_t* and uint8_t* never point
// to the same memory. The compiler may cache the buffer contents in registers
// and never re-read from memory, producing wrong results. Use std::memcpy to
// safely copy bytes into a correctly-typed local variable.

ImuPacket parse_packet(const uint8_t* buf, int len) {
    if (len < 16) {
        throw std::runtime_error("invalid packet");
    }

    // Reconstruct big-endian timestamp using only bit-shifts and OR.
    uint32_t ts = (uint32_t(buf[0]) << 24) | (uint32_t(buf[1]) << 16)
                | (uint32_t(buf[2]) <<  8) |  uint32_t(buf[3]);

    // Reconstruct each little-endian int16_t via bit manipulation.
    // buf[i] is the low byte; buf[i+1] is the high byte.
    auto le16 = [](const uint8_t* b) -> int16_t {
        return static_cast<int16_t>((uint16_t(b[1]) << 8) | uint16_t(b[0]));
    };

    ImuPacket pkt;
    pkt.timestamp_ms = ts;
    pkt.ax = le16(buf + 4);
    pkt.ay = le16(buf + 6);
    pkt.az = le16(buf + 8);
    pkt.gx = le16(buf + 10);
    pkt.gy = le16(buf + 12);
    pkt.gz = le16(buf + 14);
    return pkt;
}

int main() {
    part_a();

    std::cout << "\n";

    // Part B outputs
    std::cout << "lsb_to_ms2(16384, 2.0) = "
              << std::fixed << std::setprecision(5)
              << lsb_to_ms2(16384, 2.0) << " m/s2\n";
    std::cout << "ms_to_us(1000) = "    << ms_to_us(1000u)    << "\n";
    std::cout << "ms_to_us(4294968) = " << ms_to_us(4294968u) << "\n";
    std::cout << "is_valid_imu_header(0xA5) = "
              << static_cast<int>(is_valid_imu_header(0xA5)) << "\n";
    std::cout << "is_valid_imu_header(0xB0) = "
              << static_cast<int>(is_valid_imu_header(0xB0)) << "\n";

    // Part C — parse test packet
    uint8_t test_packet[] = {
        0x00, 0x00, 0x0F, 0xA0,  // timestamp big-endian: 4000
        0x40, 0xFE,              // ax little-endian: -448
        0x00, 0x00,              // ay: 0
        0x00, 0x08,              // az little-endian: 2048
        0xE8, 0x03,              // gx little-endian: 1000
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
