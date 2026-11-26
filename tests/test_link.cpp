/* Link tests, written before the link existed.
 *
 * A delay line is easy to write and easy to get right by luck. The three things
 * checked here are the three that a jittered, lossy line gets wrong: the delay
 * is not what the config says, a sample arrives before it was sent, or a
 * sequence that was pushed is never delivered.
 */

#include <cstdint>
#include <vector>

#include "link.h"
#include "tsim_test.h"

namespace {

constexpr std::uint32_t kPushes = 200u;

tsim::LinkSample sample_with_sequence(std::uint32_t sequence) {
    tsim::LinkSample sample;
    sample.position_rad = 0.001 * static_cast<double>(sequence);
    sample.sequence = sequence;
    sample.valid = true;
    return sample;
}

} // namespace

TSIM_TEST(link_delays_a_sample_by_exactly_its_delay) {
    /* One sample, no jitter, no loss: it arrives on the window the config names
     * and not one window earlier. */
    tsim::DelayLine line;
    line.configure(3u, 0u, 0.0, 7u);
    line.push(sample_with_sequence(1u), 0u);
    TSIM_CHECK(!line.pop(0u).valid);
    TSIM_CHECK(!line.pop(1u).valid);
    TSIM_CHECK(!line.pop(2u).valid);
    const tsim::LinkSample arrived = line.pop(3u);
    TSIM_CHECK(arrived.valid);
    TSIM_CHECK(arrived.sequence == 1u);
    TSIM_CHECK(line.delivered() == 1u);
    TSIM_CHECK(!line.pop(4u).valid);
}

TSIM_TEST(link_jitter_never_arrives_early) {
    /* With delay 10 and jitter 3, no sample may arrive before 7 windows after it
     * was pushed, whichever way the generator happens to fall. Every sequence
     * has to arrive exactly once. */
    tsim::DelayLine line;
    line.configure(10u, 3u, 0.0, 11u);
    std::vector<int> arrival(kPushes, -1);
    for (std::uint32_t window = 0u; window <= kPushes + 50u; ++window) {
        if (window < kPushes) {
            line.push(sample_with_sequence(window), window);
        }
        const tsim::LinkSample out = line.pop(window);
        if (out.valid) {
            arrival[out.sequence] = static_cast<int>(window);
        }
    }
    for (std::uint32_t sequence = 0u; sequence < kPushes; ++sequence) {
        TSIM_CHECK(arrival[sequence] >= 0);
        TSIM_CHECK(arrival[sequence] >= static_cast<int>(sequence) + 7);
    }
    TSIM_CHECK(line.dropped() == 0u);
}

TSIM_TEST(link_jitter_stays_inside_its_window_for_a_single_sample) {
    /* A single push isolates the draw from the coalescing that a continuous
     * stream produces, so the arrival window itself can be asserted. Twenty
     * seeds is enough to catch a modulus that is off by one. */
    for (std::uint32_t seed = 1u; seed <= 20u; ++seed) {
        tsim::DelayLine line;
        line.configure(10u, 3u, 0.0, seed);
        line.push(sample_with_sequence(0u), 0u);
        int arrived_at = -1;
        for (std::uint32_t window = 0u; window <= 40u; ++window) {
            if (line.pop(window).valid) {
                arrived_at = static_cast<int>(window);
                break;
            }
        }
        TSIM_CHECK(arrived_at >= 7);
        TSIM_CHECK(arrived_at <= 13);
    }
}

TSIM_TEST(link_drops_what_it_is_told_to_drop) {
    tsim::DelayLine line;
    line.configure(5u, 0u, 1.0, 3u);
    bool any_arrived = false;
    for (std::uint32_t window = 0u; window < 40u; ++window) {
        line.push(sample_with_sequence(window), window);
        any_arrived = any_arrived || line.pop(window + 5u).valid;
    }
    TSIM_CHECK(!any_arrived);
    TSIM_CHECK(line.dropped() == 40u);

    /* A lossy line loses about the fraction it was configured with. The
     * tolerance is wide because this is a property of a generator, not of a
     * physics model, and a tight band would fail on a reseed. */
    tsim::DelayLine half;
    half.configure(5u, 0u, 0.5, 5u);
    std::uint32_t delivered = 0u;
    for (std::uint32_t window = 0u; window < 4000u; ++window) {
        if (window < 2000u) {
            half.push(sample_with_sequence(window), window);
        }
        if (half.pop(window).valid) {
            delivered += 1u;
        }
    }
    TSIM_CHECK(delivered + half.dropped() == 2000u);
    TSIM_CHECK(delivered > 850u);
    TSIM_CHECK(delivered < 1150u);
}

TSIM_TEST(link_is_repeatable_for_the_same_seed) {
    /* Two links with the same seed and the same pushes have to schedule the same
     * arrivals, or a repeated harness run is not byte identical. */
    tsim::DelayLine first;
    tsim::DelayLine second;
    first.configure(12u, 4u, 0.2, 99u);
    second.configure(12u, 4u, 0.2, 99u);
    std::vector<int> first_arrival(kPushes, -1);
    std::vector<int> second_arrival(kPushes, -1);
    for (std::uint32_t window = 0u; window <= kPushes + 40u; ++window) {
        for (auto *line : {&first, &second}) {
            if (window < kPushes) {
                line->push(sample_with_sequence(window), window);
            }
        }
        const tsim::LinkSample out_first = first.pop(window);
        const tsim::LinkSample out_second = second.pop(window);
        TSIM_CHECK(out_first.valid == out_second.valid);
        if (out_first.valid) {
            first_arrival[out_first.sequence] = static_cast<int>(window);
        }
        if (out_second.valid) {
            second_arrival[out_second.sequence] = static_cast<int>(window);
        }
    }
    TSIM_CHECK(first_arrival == second_arrival);
    TSIM_CHECK(first.dropped() == second.dropped());
}

TSIM_TEST(control_link_gives_the_two_directions_different_jitter) {
    /* Two directions sharing one generator would correlate the delays, which is
     * a property no network has and a study would report as one. With the same
     * push schedule the two directions must not produce the same arrival
     * pattern. */
    tsim::LinkConfig config;
    config.delay_windows = 10u;
    config.jitter_windows = 4u;
    config.loss_probability = 0.0;
    config.seed = 1234u;
    tsim::ControlLink link;
    link.configure(config);

    std::vector<int> to_far(kPushes, -1);
    std::vector<int> to_master(kPushes, -1);
    for (std::uint32_t window = 0u; window <= kPushes + 40u; ++window) {
        if (window < kPushes) {
            link.send_to_far(sample_with_sequence(window), window);
            link.send_to_master(sample_with_sequence(window), window);
        }
        const tsim::LinkSample far_sample = link.receive_at_far(window);
        const tsim::LinkSample master_sample = link.receive_at_master(window);
        if (far_sample.valid) {
            to_far[far_sample.sequence] = static_cast<int>(window);
        }
        if (master_sample.valid) {
            to_master[master_sample.sequence] = static_cast<int>(window);
        }
    }
    TSIM_CHECK(to_far != to_master);
}
