#include "link.h"

#include <algorithm>
#include <iterator>

namespace tsim {

void DelayLine::configure(std::uint32_t delay_windows, std::uint32_t jitter_windows,
                          double loss_probability, std::uint32_t seed) {
    delay_windows_ = std::min(delay_windows, kMaxDelayWindows);
    /* Jitter wider than the delay would mean a sample can arrive before it was
     * sent, which is not a link. Clamping it is the only honest answer: dropping
     * the samples it would affect is a different link. */
    jitter_windows_ = std::min(jitter_windows, delay_windows_);
    loss_probability_ =
        (loss_probability > 0.0) ? ((loss_probability < 1.0) ? loss_probability : 1.0) : 0.0;
    rng_ = (seed != 0u) ? seed : 1u;
    reset();
}

void DelayLine::reset() {
    std::fill(std::begin(slots_), std::end(slots_), Slot{});
    cursor_ = 0u;
    delivered_ = 0u;
    dropped_ = 0u;
}

std::uint32_t DelayLine::next_random() {
    /* xorshift32, the same generator the HAL uses, so a run is repeatable and
     * a test can predict the pattern without a library. */
    std::uint32_t x = rng_;
    x ^= x << 13u;
    x ^= x >> 17u;
    x ^= x << 5u;
    rng_ = x;
    return x;
}

void DelayLine::push(const LinkSample &sample, std::uint64_t now_window) {
    const double draw = static_cast<double>(next_random() >> 8u) / 16777216.0;
    if (draw < loss_probability_) {
        dropped_ += 1u;
        return;
    }
    std::int64_t delay = static_cast<std::int64_t>(delay_windows_);
    if (jitter_windows_ > 0u) {
        const std::uint32_t span = 2u * jitter_windows_ + 1u;
        const std::int64_t offset = static_cast<std::int64_t>(next_random() % span) -
                                    static_cast<std::int64_t>(jitter_windows_);
        delay += offset;
    }
    const std::int64_t arrival =
        static_cast<std::int64_t>(now_window) + std::max<std::int64_t>(delay, 0);

    Slot &slot = slots_[cursor_ % kSlots];
    cursor_ += 1u;
    slot.sample = sample;
    slot.sample.valid = true;
    slot.arrival_window = static_cast<std::uint64_t>(arrival);
    slot.used = true;
}

LinkSample DelayLine::pop(std::uint64_t now_window) {
    Slot *newest = nullptr;
    for (Slot &slot : slots_) {
        if (!slot.used || slot.arrival_window > now_window) {
            continue;
        }
        if (newest == nullptr || slot.arrival_window > newest->arrival_window ||
            (slot.arrival_window == newest->arrival_window &&
             slot.sample.sequence >= newest->sample.sequence)) {
            newest = &slot;
        }
    }
    if (newest == nullptr) {
        return LinkSample{};
    }
    LinkSample out = newest->sample;
    newest->used = false;
    delivered_ += 1u;
    return out;
}

std::uint32_t DelayLine::delivered() const {
    return delivered_;
}

std::uint32_t DelayLine::dropped() const {
    return dropped_;
}

void ControlLink::configure(const LinkConfig &config) {
    to_far_.configure(config.delay_windows, config.jitter_windows, config.loss_probability,
                      config.seed);
    /* The other direction gets a different generator, because two directions
     * sharing a jitter pattern is a correlation a real network does not have
     * and a study would report as a property of the link. */
    to_master_.configure(config.delay_windows, config.jitter_windows, config.loss_probability,
                         config.seed * 1664525u + 1013904223u);
}

void ControlLink::reset() {
    to_far_.reset();
    to_master_.reset();
}

void ControlLink::send_to_far(const LinkSample &sample, std::uint64_t now_window) {
    to_far_.push(sample, now_window);
}

void ControlLink::send_to_master(const LinkSample &sample, std::uint64_t now_window) {
    to_master_.push(sample, now_window);
}

LinkSample ControlLink::receive_at_far(std::uint64_t now_window) {
    return to_far_.pop(now_window);
}

LinkSample ControlLink::receive_at_master(std::uint64_t now_window) {
    return to_master_.pop(now_window);
}

} // namespace tsim
