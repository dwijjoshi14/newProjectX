/* tandem_wire.h, the packet one joint sends the other one.
 *
 * Why a codec and not a struct on the socket. A struct on the wire is a struct
 * only for as long as the compiler that sent it agrees with the compiler that
 * receives it about padding and endianness, and the day the second one is not
 * the same build the link starts reading a float out of a hole. The fields are
 * written one at a time, little endian, in a stated order, so the packet is a
 * packet and not an ABI.
 *
 * The fields are also the smallest set that keeps the day 6 questions
 * answerable: where the far side is, how fast it is going, how much torque it
 * is making, and how much energy its port has taken in. The last one is what
 * makes a two port passivity observer possible at all, because a drive cannot
 * see the other end's energy any other way.
 */
#ifndef TSIM_HARNESS_TANDEM_WIRE_H
#define TSIM_HARNESS_TANDEM_WIRE_H

#include <cstddef>
#include <cstdint>

namespace tsim {

inline constexpr std::uint32_t kWireMagic = 0x314D5354u; /* "TSM1", little endian */
inline constexpr std::uint16_t kWireVersion = 1u;
inline constexpr std::uint16_t kWireFlagValid = 1u;

/* Magic, version, flags, sequence, release window, sender timestamp, then four
 * floats. Fixed so a receiver can reject a truncated packet before touching a
 * field. */
inline constexpr std::size_t kWireSampleBytes = 40u;

struct WireSample {
    std::uint32_t sequence = 0;       /* send order, count */
    std::uint32_t release_window = 0; /* window the sender says it is due, count */
    std::uint64_t stamp_ns = 0;       /* sender monotonic clock, ns */
    float position_rad = 0.0f;        /* rotor angle, rad */
    float velocity_rad_s = 0.0f;      /* rotor speed, rad/s */
    float torque_nm = 0.0f;           /* rendered torque, N m */
    float energy_j = 0.0f;            /* accumulated port energy, J */
    bool valid = true;                /* sender had a sample to send, bool */
};

/* Writes one packet. Returns bytes written, which is kWireSampleBytes, or 0 if
 * the buffer is too small. */
std::size_t encode_sample(const WireSample &sample, std::uint8_t *out, std::size_t capacity);

/* Reads one packet. False for a short buffer, a wrong magic or a version this
 * build does not speak, which are the three ways a link delivers something that
 * is not a sample. */
bool decode_sample(const std::uint8_t *in, std::size_t length, WireSample *out);

} // namespace tsim

#endif /* TSIM_HARNESS_TANDEM_WIRE_H */
