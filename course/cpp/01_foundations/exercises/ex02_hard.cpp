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
//     What is the type of c? What is its value?
//     Why doesn't it wrap to 44 (as it would with 8-bit arithmetic)?
//
//     YOUR ANSWER:
//     TODO

// Q2. Usual arithmetic conversions (signed/unsigned mixing):
//     `int neg = -1; uint32_t pos = 0; bool result = (neg < pos);`
//     What is result? Trace the implicit conversion step by step.
//
//     YOUR ANSWER:
//     TODO

// Q3. Endianness:
//     What is endianness? Is x86-64 big-endian or little-endian?
//     An IMU sends a 32-bit timestamp big-endian (MSB first).
//     How do you reconstruct it correctly on a little-endian machine?
//
//     YOUR ANSWER:
//     TODO

// Q4. Strict aliasing rule:
//     `int32_t val = *reinterpret_cast<int32_t*>(byte_buf);`
//     Why is this undefined behavior? What is the safe C++17 alternative?
//     (Research: std::memcpy, std::bit_cast)
//
//     YOUR ANSWER:
//     TODO

// Q5. Loop trap:
//     `for (uint8_t i = 0; i < 256; ++i) { /* ... */ }`
//     What happens and why? Trace using integer promotion rules.
//
//     YOUR ANSWER:
//     TODO

// Q6. constexpr vs const:
//     (a) Can a constexpr function be called at runtime?
//     (b) Can a constexpr function contain if-statements and for-loops (C++17)?
//     (c) What is the fundamental difference between constexpr and const?
//
//     YOUR ANSWER:
//     TODO

// ─────────────────────────────────────────────────────────────────────────────
// PART A — Predict the output of each snippet BEFORE compiling.
//
// Write your prediction as a comment on the line marked PREDICT, then compile
// and compare. Explain any surprises.
// ─────────────────────────────────────────────────────────────────────────────

#include <iostream>
#include <iomanip>
#include <cstdint>
#include <cstring>
#include <stdexcept>

void part_a() {
    std::cout << std::fixed << std::setprecision(5);

    // Snippet 1 — integer promotion on uint8_t addition
    // PREDICT: TODO (what does this print?)
    {
        uint8_t a = 200, b = 100;
        auto c = a + b;
        std::cout << "snippet1: " << c << "\n";
    }

    // Snippet 2 — signed/unsigned comparison trap
    // PREDICT: TODO
    {
        int neg = -1;
        uint32_t pos = 0;
        std::cout << "snippet2: " << (neg < pos ? "less" : "not less") << "\n";
    }

    // Snippet 3 — uint8_t overflow
    // PREDICT: TODO
    {
        uint8_t count = 255;
        count += 1;
        std::cout << "snippet3: " << static_cast<int>(count) << "\n";
    }

    // Snippet 4 — int16_t to double division (note: integer / floating-point)
    // PREDICT: TODO
    {
        int16_t raw = -32000;
        double scaled = raw / 32768.0;
        std::cout << "snippet4: " << scaled << "\n";
    }

    // Snippet 5 — two comparisons: one with cast, one without
    // PREDICT snippet5a (with cast): TODO
    // PREDICT snippet5b (without cast): TODO
    {
        unsigned int limit = 3;
        int idx = -1;
        std::cout << "snippet5a: " << (idx < static_cast<int>(limit) ? "A" : "B") << "\n";
        std::cout << "snippet5b: " << (idx < limit                   ? "C" : "D") << "\n";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// PART B — constexpr sensor utility functions
//
// Implement each function AND add the given static_assert to verify
// compile-time evaluation.
// ─────────────────────────────────────────────────────────────────────────────

// B1. Convert raw ADC value to acceleration in m/s²
//     Formula: (raw / 32768.0) * full_scale_g * 9.80665
//     The sensor full-scale is specified in units of g.
//
// TODO: Implement
constexpr double lsb_to_ms2(int16_t raw, double full_scale_g) {
    return 0.0; // TODO
}
// After implementing, uncomment to verify compile-time evaluation:
// static_assert(lsb_to_ms2(0, 2.0) == 0.0, "lsb_to_ms2(0, 2.0) should be 0");

// B2. Convert milliseconds to microseconds, saturating on overflow.
//     A naive `ms * 1000` overflows for large ms values.
//     Safe version: check before multiplying.
//     Use: return ms > (UINT32_MAX / 1000u) ? UINT32_MAX : ms * 1000u;
//
// TODO: Implement
constexpr uint32_t ms_to_us(uint32_t ms) {
    return 0u; // TODO
}
// After implementing, uncomment to verify compile-time evaluation:
// static_assert(ms_to_us(1000u) == 1000000u, "ms_to_us(1000) should be 1000000");

// B3. Validate an IMU start byte: the high nibble must equal 0xA.
//     Use ONLY bitwise operations — no division, no multiplication.
//
// TODO: Implement
constexpr bool is_valid_imu_header(uint8_t b) {
    return false; // TODO
}

// ─────────────────────────────────────────────────────────────────────────────
// PART C — Safe IMU binary packet parser
//
// Wire format (16 bytes total):
//   Bytes [0..3]:   timestamp_ms  — BIG-ENDIAN (MSB first)
//   Bytes [4..5]:   ax            — little-endian int16_t
//   Bytes [6..7]:   ay            — little-endian int16_t
//   Bytes [8..9]:   az            — little-endian int16_t
//   Bytes [10..11]: gx            — little-endian int16_t
//   Bytes [12..13]: gy            — little-endian int16_t
//   Bytes [14..15]: gz            — little-endian int16_t
//
// Rules:
//   - Throw std::runtime_error("invalid packet") if len < 16
//   - Reconstruct timestamp using ONLY bit-shifts and OR (not memcpy/reinterpret_cast)
//   - Reconstruct each int16_t using little-endian reconstruction
//   - Add a comment explaining WHY reinterpret_cast<int32_t*>(buf) is UB here
// ─────────────────────────────────────────────────────────────────────────────

struct ImuPacket {
    uint32_t timestamp_ms;
    int16_t  ax, ay, az;   // raw ADC, ±4g scale
    int16_t  gx, gy, gz;   // raw ADC, ±500 dps scale
};

// TODO: Why is reinterpret_cast<int32_t*>(buf) UB here?
// Explain strict aliasing in a comment below:
//
// TODO: YOUR ANSWER

ImuPacket parse_packet(const uint8_t* buf, int len) {
    // TODO: implement
    //   1. Check len >= 16, throw if not
    //   2. Reconstruct timestamp (big-endian):
    //      uint32_t ts = (uint32_t(buf[0])<<24) | (uint32_t(buf[1])<<16)
    //                  | (uint32_t(buf[2])<<8)  |  uint32_t(buf[3]);
    //   3. Reconstruct each int16_t (little-endian):
    //      int16_t ax = int16_t((uint16_t(buf[5])<<8) | uint16_t(buf[4]));
    //      (note: buf[i+1] is the high byte, buf[i] is the low byte)
    //   4. Fill and return ImuPacket
    return ImuPacket{}; // TODO
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
    // Wire bytes encode: timestamp=4000ms, ax=-448, ay=0, az=2048,
    //                    gx=1000, gy=0, gz=0
    // Verify by hand:
    //   0x00000FA0 = 4000
    //   0xFE40 (little-endian bytes: 0x40, 0xFE) = -448 signed
    //   0x0800 (little-endian bytes: 0x00, 0x08) = 2048
    //   0x03E8 (little-endian bytes: 0xE8, 0x03) = 1000
    uint8_t test_packet[] = {
        0x00, 0x00, 0x0F, 0xA0,  // timestamp big-endian
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
