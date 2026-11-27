/* Wire tests, written before the codec existed.
 *
 * A packet codec fails in three ways that a round trip test alone does not
 * catch: it loses the sign of a float, it accepts a packet from a different
 * build, or it reads past the end of a short one. All three are tested here,
 * and the round trip is tested on values that are not round numbers so that a
 * field written into the wrong offset cannot pass.
 */

#include <cstdint>

#include "tandem_wire.h"
#include "tsim_test.h"

namespace {

tsim::WireSample sample_with_every_field_different() {
    tsim::WireSample sample;
    sample.sequence = 123456u;
    sample.release_window = 40123u;
    sample.stamp_ns = 987654321012ull;
    sample.position_rad = -0.012345f;
    sample.velocity_rad_s = 47.25f;
    sample.torque_nm = -2.71828f;
    sample.energy_j = 1234.5f;
    sample.valid = true;
    return sample;
}

} // namespace

TSIM_TEST(wire_round_trips_every_field) {
    const tsim::WireSample sent = sample_with_every_field_different();
    std::uint8_t buffer[tsim::kWireSampleBytes]{};
    TSIM_CHECK(tsim::encode_sample(sent, buffer, sizeof buffer) == tsim::kWireSampleBytes);

    tsim::WireSample received;
    TSIM_CHECK(tsim::decode_sample(buffer, sizeof buffer, &received));
    TSIM_CHECK(received.sequence == sent.sequence);
    TSIM_CHECK(received.release_window == sent.release_window);
    TSIM_CHECK(received.stamp_ns == sent.stamp_ns);
    TSIM_CHECK(received.position_rad == sent.position_rad);
    TSIM_CHECK(received.velocity_rad_s == sent.velocity_rad_s);
    TSIM_CHECK(received.torque_nm == sent.torque_nm);
    TSIM_CHECK(received.energy_j == sent.energy_j);
    TSIM_CHECK(received.valid);
    /* The packet is fixed size, because a receiver that sizes its buffer from
     * the header is a receiver that can be told to overrun it. */
    TSIM_CHECK(tsim::kWireSampleBytes == 40u);
}

TSIM_TEST(wire_refuses_a_short_packet) {
    const tsim::WireSample sent = sample_with_every_field_different();
    std::uint8_t buffer[tsim::kWireSampleBytes]{};
    (void)tsim::encode_sample(sent, buffer, sizeof buffer);

    tsim::WireSample received;
    TSIM_CHECK(!tsim::decode_sample(buffer, tsim::kWireSampleBytes - 1u, &received));
    TSIM_CHECK(!tsim::decode_sample(buffer, 4u, &received));
    TSIM_CHECK(!tsim::decode_sample(buffer, 0u, &received));
}

TSIM_TEST(wire_refuses_another_protocol) {
    const tsim::WireSample sent = sample_with_every_field_different();
    std::uint8_t buffer[tsim::kWireSampleBytes]{};
    (void)tsim::encode_sample(sent, buffer, sizeof buffer);

    tsim::WireSample received;
    std::uint8_t wrong_magic[tsim::kWireSampleBytes]{};
    for (std::size_t i = 0; i < sizeof wrong_magic; ++i) {
        wrong_magic[i] = buffer[i];
    }
    wrong_magic[0] = 0x00u;
    TSIM_CHECK(!tsim::decode_sample(wrong_magic, sizeof wrong_magic, &received));

    std::uint8_t wrong_version[tsim::kWireSampleBytes]{};
    for (std::size_t i = 0; i < sizeof wrong_version; ++i) {
        wrong_version[i] = buffer[i];
    }
    wrong_version[4] = 99u;
    TSIM_CHECK(!tsim::decode_sample(wrong_version, sizeof wrong_version, &received));

    TSIM_CHECK(tsim::decode_sample(buffer, sizeof buffer, &received));
}

TSIM_TEST(wire_reports_a_buffer_too_small_to_encode_into) {
    const tsim::WireSample sent = sample_with_every_field_different();
    std::uint8_t buffer[tsim::kWireSampleBytes]{};
    TSIM_CHECK(tsim::encode_sample(sent, buffer, tsim::kWireSampleBytes - 1u) == 0u);
}

TSIM_TEST(wire_clears_the_valid_flag_across_the_link) {
    /* A sender with no sample still has to say so, because the alternative is
     * the receiver holding an angle from a cycle ago and rendering against it
     * as if it were now. */
    tsim::WireSample sent = sample_with_every_field_different();
    sent.valid = false;
    std::uint8_t buffer[tsim::kWireSampleBytes]{};
    (void)tsim::encode_sample(sent, buffer, sizeof buffer);
    tsim::WireSample received;
    TSIM_CHECK(tsim::decode_sample(buffer, sizeof buffer, &received));
    TSIM_CHECK(!received.valid);
}
