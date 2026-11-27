#include "tandem_wire.h"

namespace tsim {
namespace {

/* Little endian, byte at a time. A memcpy of a float into a packet is portable
 * between two builds of the same compiler and nothing else, and the point of a
 * codec is that the packet does not depend on who wrote it. */
/* The shifts are done on an explicitly unsigned value of the widest type in
 * play. A uint16 shifted by an unsigned count is promoted to int first, and a
 * signed shift is both the wrong thing to reason about at a packet boundary and
 * something clang-tidy rejects. */
void put_u16(std::uint8_t *out, std::size_t offset, std::uint16_t value) {
    const std::uint32_t bits = value;
    out[offset] = static_cast<std::uint8_t>(bits & 0xFFu);
    out[offset + 1u] = static_cast<std::uint8_t>((bits >> 8u) & 0xFFu);
}

void put_u32(std::uint8_t *out, std::size_t offset, std::uint32_t value) {
    for (std::size_t i = 0; i < 4u; ++i) {
        const std::uint32_t shift = static_cast<std::uint32_t>(8u * i);
        out[offset + i] = static_cast<std::uint8_t>((value >> shift) & 0xFFu);
    }
}

void put_u64(std::uint8_t *out, std::size_t offset, std::uint64_t value) {
    for (std::size_t i = 0; i < 8u; ++i) {
        const std::uint32_t shift = static_cast<std::uint32_t>(8u * i);
        out[offset + i] = static_cast<std::uint8_t>((value >> shift) & 0xFFu);
    }
}

std::uint16_t get_u16(const std::uint8_t *in, std::size_t offset) {
    const std::uint32_t low = in[offset];
    const std::uint32_t high = in[offset + 1u];
    return static_cast<std::uint16_t>(low | (high << 8u));
}

std::uint32_t get_u32(const std::uint8_t *in, std::size_t offset) {
    std::uint32_t value = 0u;
    for (std::size_t i = 0; i < 4u; ++i) {
        const std::uint32_t shift = static_cast<std::uint32_t>(8u * i);
        value |= static_cast<std::uint32_t>(in[offset + i]) << shift;
    }
    return value;
}

std::uint64_t get_u64(const std::uint8_t *in, std::size_t offset) {
    std::uint64_t value = 0u;
    for (std::size_t i = 0; i < 8u; ++i) {
        const std::uint32_t shift = static_cast<std::uint32_t>(8u * i);
        value |= static_cast<std::uint64_t>(in[offset + i]) << shift;
    }
    return value;
}

/* A float is sent as the bit pattern the machine has for it. IEEE 754 single
 * precision is what a Cortex M4F and an x86 both use, and this link has never
 * claimed to be portable to anything else. */
void put_f32(std::uint8_t *out, std::size_t offset, float value) {
    std::uint32_t bits = 0u;
    static_assert(sizeof bits == sizeof value, "a float is not 32 bits here");
    unsigned char bytes[sizeof bits];
    const unsigned char *source = reinterpret_cast<const unsigned char *>(&value);
    for (std::size_t i = 0; i < sizeof bits; ++i) {
        bytes[i] = source[i];
    }
    for (std::size_t i = 0; i < sizeof bits; ++i) {
        const std::uint32_t shift = static_cast<std::uint32_t>(8u * i);
        bits |= static_cast<std::uint32_t>(bytes[i]) << shift;
    }
    put_u32(out, offset, bits);
}

float get_f32(const std::uint8_t *in, std::size_t offset) {
    const std::uint32_t bits = get_u32(in, offset);
    float value = 0.0f;
    unsigned char *destination = reinterpret_cast<unsigned char *>(&value);
    for (std::size_t i = 0; i < sizeof bits; ++i) {
        const std::uint32_t shift = static_cast<std::uint32_t>(8u * i);
        destination[i] = static_cast<unsigned char>((bits >> shift) & 0xFFu);
    }
    return value;
}

} // namespace

std::size_t encode_sample(const WireSample &sample, std::uint8_t *out, std::size_t capacity) {
    if (out == nullptr || capacity < kWireSampleBytes) {
        return 0u;
    }
    put_u32(out, 0u, kWireMagic);
    put_u16(out, 4u, kWireVersion);
    put_u16(out, 6u, sample.valid ? kWireFlagValid : 0u);
    put_u32(out, 8u, sample.sequence);
    put_u32(out, 12u, sample.release_window);
    put_u64(out, 16u, sample.stamp_ns);
    put_f32(out, 24u, sample.position_rad);
    put_f32(out, 28u, sample.velocity_rad_s);
    put_f32(out, 32u, sample.torque_nm);
    put_f32(out, 36u, sample.energy_j);
    return kWireSampleBytes;
}

bool decode_sample(const std::uint8_t *in, std::size_t length, WireSample *out) {
    if (in == nullptr || out == nullptr || length < kWireSampleBytes) {
        return false;
    }
    if (get_u32(in, 0u) != kWireMagic || get_u16(in, 4u) != kWireVersion) {
        return false;
    }
    out->valid = (get_u16(in, 6u) & kWireFlagValid) != 0u;
    out->sequence = get_u32(in, 8u);
    out->release_window = get_u32(in, 12u);
    out->stamp_ns = get_u64(in, 16u);
    out->position_rad = get_f32(in, 24u);
    out->velocity_rad_s = get_f32(in, 28u);
    out->torque_nm = get_f32(in, 32u);
    out->energy_j = get_f32(in, 36u);
    return true;
}

} // namespace tsim
