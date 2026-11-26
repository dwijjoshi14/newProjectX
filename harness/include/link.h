/* link.h, the wire between two joints, with the three things a wire has that
 * a shared variable does not: delay, jitter and loss.
 *
 * Why it is its own class and not a std::deque inside the scenario: the link is
 * the thing being studied on day 5. A delay line written inline is the kind of
 * code that is right by luck, and the test that its delay is what it says it is
 * needs something with a name.
 *
 * Time here is in control windows, not seconds, because everything on the
 * control path is a whole number of 50 us ticks and a link that scheduled on
 * something finer would make the harness non deterministic. One window is 50 us,
 * so a 2 ms delay is 40 windows.
 *
 * Arrival is stored per sample rather than read from a delay-specific ring: a
 * jittered sample can arrive before one that was pushed earlier, so the receiver
 * has to take the newest arrived sample and not the next one in order. That is
 * also what a real socket does.
 */
#ifndef TSIM_HARNESS_LINK_H
#define TSIM_HARNESS_LINK_H

#include <cstdint>

namespace tsim {

/* One sample of one side of a bilateral pair, as it crosses the link. */
struct LinkSample {
    double position_rad = 0.0;   /* joint angle, rad */
    double velocity_rad_s = 0.0; /* joint speed, rad/s */
    std::uint64_t sequence = 0;  /* push order, count */
    bool valid = false;          /* false means nothing has arrived */
};

struct LinkConfig {
    std::uint32_t delay_windows = 40;  /* nominal one way delay, windows */
    std::uint32_t jitter_windows = 20; /* +- this, uniform, windows */
    double loss_probability = 0.005;   /* fraction of samples dropped */
    std::uint32_t seed = 0x1234567u;   /* one seed per direction, offset inside */
};

/* One direction of a link. */
class DelayLine {
  public:
    /* Above this the ring wraps before the sample is due and the delay quietly
     * becomes something else, so it is a hard limit rather than a comment. */
    static constexpr std::uint32_t kMaxDelayWindows = 200;

    void configure(std::uint32_t delay_windows, std::uint32_t jitter_windows,
                   double loss_probability, std::uint32_t seed);

    void reset();

    /* Queue one sample, to arrive at now_window plus its own delay. */
    void push(const LinkSample &sample, std::uint64_t now_window);

    /* Newest sample whose arrival is not in the future, invalid if none. */
    LinkSample pop(std::uint64_t now_window);

    std::uint32_t delivered() const;
    std::uint32_t dropped() const;

  private:
    static constexpr std::uint32_t kSlots = 256;

    struct Slot {
        LinkSample sample{};
        std::uint64_t arrival_window = 0;
        bool used = false;
    };

    std::uint32_t next_random();

    Slot slots_[kSlots]{};
    std::uint32_t delay_windows_ = 0;
    std::uint32_t jitter_windows_ = 0;
    double loss_probability_ = 0.0;
    std::uint32_t rng_ = 1u;
    std::uint32_t cursor_ = 0;
    std::uint32_t delivered_ = 0;
    std::uint32_t dropped_ = 0;
};

/* Both directions. Named by where the sample is going, not where it came from,
 * because that is the side that has to do something with it. */
class ControlLink {
  public:
    void configure(const LinkConfig &config);
    void reset();

    void send_to_far(const LinkSample &sample, std::uint64_t now_window);
    void send_to_master(const LinkSample &sample, std::uint64_t now_window);

    LinkSample receive_at_far(std::uint64_t now_window);
    LinkSample receive_at_master(std::uint64_t now_window);

  private:
    DelayLine to_far_;
    DelayLine to_master_;
};

} // namespace tsim

#endif /* TSIM_HARNESS_LINK_H */
