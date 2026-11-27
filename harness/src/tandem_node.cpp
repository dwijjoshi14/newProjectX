/* tandem_node.cpp, one joint of a bilateral pair, as its own process.
 *
 * Day 5 put two joints in one process and said plainly that the far joint had no
 * drive, because hal.h is a single MCU contract and one process can only host one
 * firmware instance. This file is the answer that was deferred: two programs, each
 * with its own plant, its own encoder and its own unmodified firmware, exchanging
 * a sample over a real datagram socket.
 *
 * What that buys and what it costs. It buys a second drive, and with it the thing
 * day 5 could not measure: a pair where both ends render a spring and both ends
 * can generate. It costs determinism. The socket is the operating system's and
 * the two schedules are independent, so a run of this binary is not byte
 * identical to the next one. Nothing in this project's determinism claim covers
 * this file; the figures it produces are stamped with the commit.
 *
 * The timing, and why it is a lockstep and not a clock. A 1 MHz switching plant
 * with seven pole pairs costs about 67 us of this machine's time per 50 us
 * control window, and the machine has two cores for two nodes and the launcher.
 * A pair cannot run at true real time here. Pacing both nodes to one wall clock
 * was tried first and measured: the two timelines separated by up to 14.5 ms,
 * which turns a 2 ms link into a 15 ms one, and a steering loop on the packet
 * timestamps only moved the oscillation around. So the nodes lock step window by
 * window instead: at the top of window k each sends its current state, then waits
 * until the peer has also reached window k, then runs the window. Neither can get
 * more than a window ahead, no clock is read for scheduling, and the link delay
 * is a number the study sets. The transport is real, its latency is measured and
 * reported, and the lockstep is the day's largest cut, written down in
 * docs/spec.md rather than hidden.
 *
 * Both ends are symmetric: each renders stiffness times (peer angle minus own
 * angle), which is a spring between the two, and each one's reference is the
 * other's position as it arrived. The excitation is the initial offset, so the
 * pair starts stretched and rings down, or does not.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <exception>
#include <filesystem>
#include <string>
#include <thread>

#include "firmware.h"
#include "hal.h"
#include "link.h"
#include "sim_hal.h"
#include "sim_loop.h"
#include "tandem_wire.h"
#include "tsim/plant.h"
#include "tsim/plant_params.h"
#include "tsim/switching_plant.h"
#include "udp_link.h"

namespace {

/* Encoder noise of the parameter table, rad RMS. Both ends commission from this
 * number, so a quiet encoder here would be a different and flattering joint. */
constexpr double kEncoderNoiseRad = 8.7266e-4;
/* The gate dead time the board config and the plant both get. */
constexpr double kDeadTimeS = 5.0e-7;
/* How long a node waits for the peer before deciding the peer is gone. Five of
 * these in a row ends the run instead of hanging it. */
constexpr int kPeerWaitMs = 20;
constexpr std::uint32_t kMaxPeerTimeouts = 5u;

struct NodeConfig {
    bool far = false;
    std::uint16_t local_port = 47001;
    std::uint16_t peer_port = 47002;
    std::uint32_t delay_windows = 40;          /* one way delay, 50 us windows */
    double stiffness_nm_per_rad = 70.0;        /* rendered coupling, N m/rad */
    double damping_nm_s_per_rad = 0.02;        /* rendered damping, N m s/rad */
    double gap_rad = 0.002;                    /* initial offset, rad */
    double shaft_stiffness_nm_per_rad = 250.0; /* the joint's own shaft, N m/rad */
    std::uint64_t ticks = 10000;               /* 0.5 s at 20 kHz */
    std::uint64_t log_every = 5;
    int controller = 0; /* 0 off, 1 one port, 2 two port */
    const char *out_path = "";
};

struct RunResult {
    double excess_min_j = 0.0;   /* combined excess, which is what it damps on */
    double energy_min_j = 0.0;   /* this port's own accumulated energy */
    double energy_max_j = 0.0;   /* this port's peak delivery, J, positive */
    double energy_final_j = 0.0; /* where this port's accumulator ended, J */
    double gap_max_rad = 0.0;    /* largest own to peer angle difference, rad */
    double gap_final_rad = 0.0;  /* where that difference ended, rad */
    double peer_energy_j = 0.0;
    double damping_max_nm_s_per_rad = 0.0;
    double peak_early_rad_s = 0.0;
    double peak_late_rad_s = 0.0;
    double latency_sum_us = 0.0;
    double latency_max_us = 0.0;
    std::uint64_t latency_count = 0u;
    std::uint64_t delivered = 0u;
    std::uint64_t malformed = 0u;
    std::uint64_t saturated = 0u;
    std::uint64_t peer_timeouts = 0u;
    std::int64_t skew_max_windows = 0;
    bool send_error = false;
};

std::uint64_t monotonic_ns() {
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    return static_cast<std::uint64_t>(now.tv_sec) * 1000000000ull +
           static_cast<std::uint64_t>(now.tv_nsec);
}

const char *controller_name(int controller) {
    switch (controller) {
    case 1:
        return "one-port";
    case 2:
        return "two-port";
    default:
        return "off";
    }
}

bool parse_controller(const char *text, int *out_controller) {
    if (std::strcmp(text, "off") == 0) {
        *out_controller = 0;
        return true;
    }
    if (std::strcmp(text, "one-port") == 0) {
        *out_controller = 1;
        return true;
    }
    if (std::strcmp(text, "two-port") == 0) {
        *out_controller = 2;
        return true;
    }
    return false;
}

void print_usage(const char *program) {
    std::fprintf(stderr,
                 "usage: %s --role master|far --local-port N --peer-port N\n"
                 "          [--delay-ms X] [--stiffness X] [--damping X] [--gap-rad X]\n"
                 "          [--shaft-stiffness X] [--controller off|one-port|two-port]\n"
                 "          [--ticks N] [--log-every N] [--out PATH]\n",
                 program);
}

bool parse_options(int argc, char **argv, NodeConfig *config) {
    for (int i = 1; i < argc; ++i) {
        const char *argument = argv[i];
        const bool has_value = (i + 1 < argc);
        if (std::strcmp(argument, "--role") == 0 && has_value) {
            const char *role = argv[++i];
            if (std::strcmp(role, "master") == 0) {
                config->far = false;
            } else if (std::strcmp(role, "far") == 0) {
                config->far = true;
            } else {
                std::fprintf(stderr, "unknown role: %s\n", role);
                return false;
            }
        } else if (std::strcmp(argument, "--local-port") == 0 && has_value) {
            config->local_port = static_cast<std::uint16_t>(std::strtoul(argv[++i], nullptr, 10));
        } else if (std::strcmp(argument, "--peer-port") == 0 && has_value) {
            config->peer_port = static_cast<std::uint16_t>(std::strtoul(argv[++i], nullptr, 10));
        } else if (std::strcmp(argument, "--delay-ms") == 0 && has_value) {
            const double delay_ms = std::strtod(argv[++i], nullptr);
            config->delay_windows = static_cast<std::uint32_t>(
                std::lround((delay_ms > 0.0 ? delay_ms : 0.0) * 1.0e-3 / tsim::kControlPeriodS));
        } else if (std::strcmp(argument, "--stiffness") == 0 && has_value) {
            config->stiffness_nm_per_rad = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(argument, "--damping") == 0 && has_value) {
            config->damping_nm_s_per_rad = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(argument, "--gap-rad") == 0 && has_value) {
            config->gap_rad = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(argument, "--shaft-stiffness") == 0 && has_value) {
            config->shaft_stiffness_nm_per_rad = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(argument, "--controller") == 0 && has_value) {
            if (!parse_controller(argv[++i], &config->controller)) {
                std::fprintf(stderr, "controller wants off, one-port or two-port\n");
                return false;
            }
        } else if (std::strcmp(argument, "--ticks") == 0 && has_value) {
            config->ticks = std::strtoull(argv[++i], nullptr, 10);
        } else if (std::strcmp(argument, "--log-every") == 0 && has_value) {
            config->log_every = std::strtoull(argv[++i], nullptr, 10);
        } else if (std::strcmp(argument, "--out") == 0 && has_value) {
            config->out_path = argv[++i];
        } else if (std::strcmp(argument, "--help") == 0) {
            print_usage(argv[0]);
            std::exit(0);
        } else {
            print_usage(argv[0]);
            return false;
        }
    }
    if (config->ticks == 0u) {
        std::fprintf(stderr, "--ticks wants a nonzero count\n");
        return false;
    }
    if (config->log_every == 0u) {
        config->log_every = 1u;
    }
    return true;
}

} // namespace

int main(int argc, char **argv) {
    NodeConfig config;
    if (!parse_options(argc, argv, &config)) {
        return 2;
    }

    std::string out_path = config.out_path;
    if (out_path.empty()) {
        out_path = config.far ? "data/run_latest/day6_far.csv" : "data/run_latest/day6_master.csv";
    }

    tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    params.mechanics = tsim::mechanics_params_hand();
    params.mechanics.coupling_stiffness_nm_per_rad = config.shaft_stiffness_nm_per_rad;

    tsim::SwitchingPlant plant(params, tsim::inverter_params_realistic());
    const double initial_angle_rad = config.far ? config.gap_rad : 0.0;
    plant.reset(initial_angle_rad);

    sim_hal_attach(plant);
    sim_hal_set_board_dead_time_s(static_cast<float>(kDeadTimeS));
    sim_hal_set_encoder_noise_rad(static_cast<float>(kEncoderNoiseRad));
    sim_hal_set_imperfections(true);
    /* A board's converter runs before the bridge is enabled and the firmware
     * averages that first conversion for its offsets. */
    sim_hal_latch_sample(plant.sample_electrical(), 0u);

    firmware_init();
    firmware_set_passivity_control(config.controller != 0);
    firmware_set_impedance(static_cast<float>(config.stiffness_nm_per_rad),
                           static_cast<float>(config.damping_nm_s_per_rad));
    firmware_set_mode(FIRMWARE_MODE_IMPEDANCE);
    /* Hold where the shaft already is until the first packet says otherwise. A
     * jump to the peer's angle on the first tick would be a step neither drive
     * could tell from a real one. */
    firmware_set_position_reference_rad(static_cast<float>(plant.mechanical().theta_rotor_rad));

    tsim::UdpSocket socket;
    std::string error;
    if (!socket.open(config.local_port, &error)) {
        std::fprintf(stderr, "cannot bind port %u: %s\n", static_cast<unsigned>(config.local_port),
                     error.c_str());
        return 1;
    }
    const std::uint16_t bound_port = socket.local_port();

    /* The delay line imposes the link's delay and nothing else: no jitter and no
     * loss, because the transport supplies both and a socket's drops are not
     * repeatable. Samples are pushed with the release window the sender named, so
     * a sample is due `delay_windows` after it was sent, measured on the sender's
     * own window counter. */
    tsim::DelayLine line;
    line.configure(0u, 0u, 0.0, 1u);

    const std::filesystem::path csv_path(out_path);
    try {
        if (!csv_path.parent_path().empty()) {
            std::filesystem::create_directories(csv_path.parent_path());
        }
    } catch (const std::exception &failure) {
        std::fprintf(stderr, "cannot create %s: %s\n", csv_path.string().c_str(), failure.what());
        return 1;
    }
    std::FILE *csv = std::fopen(out_path.c_str(), "w");
    if (csv == nullptr) {
        std::perror("fopen");
        return 1;
    }
    std::fprintf(csv, "tick,time_ms,theta_own_rad,theta_peer_rad,omega_own_rad_s,torque_own_nm,"
                      "iq_a,energy_j,excess_j,peer_energy_j,damping_nm_s_per_rad,latency_us,"
                      "skew_windows\n");

    RunResult result;
    double peer_position_rad = initial_angle_rad;
    double peer_energy_j = 0.0;
    std::uint32_t sequence = 0u;
    std::uint32_t newest_release = 0u;
    bool any_received = false;
    bool peer_gone = false;
    const std::uint64_t early_end = std::max<std::uint64_t>(config.ticks / 10u, 1u);
    const std::uint64_t late_start = config.ticks - config.ticks / 10u;

    /* Take one packet off the socket, whatever it is, and put it in the delay
     * line. Returns false when the socket is empty. Shared by the drain and the
     * lockstep wait so the two cannot disagree about what a packet means. */
    auto take_packet = [&](int wait_ms, std::uint64_t tick) -> bool {
        std::uint8_t buffer[256];
        const int length = (wait_ms > 0) ? socket.receive_waiting(buffer, sizeof buffer, wait_ms)
                                         : socket.receive(buffer, sizeof buffer);
        if (length <= 0) {
            return false;
        }
        tsim::WireSample wire;
        if (!tsim::decode_sample(buffer, static_cast<std::size_t>(length), &wire)) {
            result.malformed += 1u;
            return true;
        }
        const double latency_us = static_cast<double>(monotonic_ns() - wire.stamp_ns) * 1.0e-3;
        result.latency_sum_us += latency_us;
        result.latency_max_us = std::max(result.latency_max_us, latency_us);
        result.latency_count += 1u;

        /* The peer's release window against ours, after the delay it asked for
         * is taken off. Under the lockstep this is the link's whole timing
         * error, and it is measured rather than assumed. */
        const std::int64_t skew = static_cast<std::int64_t>(wire.release_window) -
                                  static_cast<std::int64_t>(tick + config.delay_windows);
        if (tick >= 1000u) {
            result.skew_max_windows =
                std::max<std::int64_t>(result.skew_max_windows, (skew < 0) ? -skew : skew);
        }

        tsim::LinkSample sample;
        sample.position_rad = static_cast<double>(wire.position_rad);
        sample.velocity_rad_s = static_cast<double>(wire.velocity_rad_s);
        sample.energy_j = static_cast<double>(wire.energy_j);
        sample.sequence = wire.sequence;
        sample.valid = wire.valid;
        line.push(sample, wire.release_window);
        newest_release = std::max(newest_release, wire.release_window);
        any_received = true;
        return true;
    };

    for (std::uint64_t tick = 0u; tick < config.ticks; ++tick) {
        const firmware_status_t *status = firmware_status();
        const tsim::MechanicalState mechanical = plant.mechanical();

        /* 1. Send what this side is now, before waiting. Before, because a
         * lockstep in which both sides wait first is a deadlock, and because a
         * drive sends the state it holds at the start of its window: the sample
         * is one window old when it arrives and the delay line accounts for
         * exactly the delay, not the delay plus one. */
        tsim::WireSample outgoing;
        outgoing.sequence = sequence;
        sequence += 1u;
        outgoing.release_window = static_cast<std::uint32_t>(tick + config.delay_windows);
        outgoing.stamp_ns = monotonic_ns();
        outgoing.position_rad = static_cast<float>(mechanical.theta_rotor_rad);
        outgoing.velocity_rad_s = static_cast<float>(mechanical.omega_rotor_rad_s);
        outgoing.torque_nm = status->torque_ref_nm;
        outgoing.energy_j = status->passivity_energy_j;
        std::uint8_t packet[tsim::kWireSampleBytes]{};
        (void)tsim::encode_sample(outgoing, packet, sizeof packet);
        if (!socket.send_to(config.peer_port, packet, sizeof packet, &error) &&
            !result.send_error) {
            result.send_error = true;
            std::fprintf(stderr, "send failed: %s\n", error.c_str());
        }

        /* 2. Take in what has already arrived. */
        while (take_packet(0, tick)) {
        }

        /* 3. Wait until the peer has reached this window. This is the lockstep:
         * neither node can be more than one window ahead, no wall clock is
         * involved, and the pair advances at the speed of the slower one. */
        const std::uint32_t wanted = static_cast<std::uint32_t>(tick + config.delay_windows);
        std::uint32_t timeouts = 0u;
        while (!any_received || newest_release < wanted) {
            if (!take_packet(kPeerWaitMs, tick)) {
                result.peer_timeouts += 1u;
                timeouts += 1u;
                if (timeouts >= kMaxPeerTimeouts) {
                    peer_gone = true;
                    break;
                }
            }
        }
        if (peer_gone) {
            std::fprintf(stderr, "peer stopped answering at window %llu\n",
                         static_cast<unsigned long long>(tick));
            break;
        }

        /* 4. Whatever is due now is the reference, and the energy that came with
         * it is the energy the far port had when it sent that angle. Using the
         * newest energy against an older angle would charge one sample's work to
         * another sample's position. */
        const tsim::LinkSample due = line.pop(tick);
        if (due.valid) {
            peer_position_rad = due.position_rad;
            peer_energy_j = due.energy_j;
            result.delivered += 1u;
            result.peer_energy_j = peer_energy_j;
        }
        firmware_set_position_reference_rad(static_cast<float>(peer_position_rad));
        firmware_set_link_energy_j((config.controller == 2) ? static_cast<float>(peer_energy_j)
                                                            : 0.0f);

        /* 5. One control window, with the drive's own tick inside it. */
        sim_run_window(plant, tick);

        result.excess_min_j =
            std::min(result.excess_min_j, static_cast<double>(status->passivity_excess_j));
        result.energy_min_j =
            std::min(result.energy_min_j, static_cast<double>(status->passivity_energy_j));
        result.energy_max_j =
            std::max(result.energy_max_j, static_cast<double>(status->passivity_energy_j));
        result.damping_max_nm_s_per_rad =
            std::max(result.damping_max_nm_s_per_rad,
                     static_cast<double>(status->passivity_damping_nm_s_per_rad));
        if (status->passivity_damping_nm_s_per_rad > 49.9f) {
            result.saturated += 1u;
        }

        /* The gap between the two joints. A bilateral pair fails by failing to
         * close it: a drive that has railed its current against a hand it cannot
         * hold drifts away from the peer and the angle ratio is the only column
         * that shows it. The speed ratio used since day 5 does not, because a
         * slow pull apart has small speeds in both windows. */
        const double gap_rad = std::fabs(mechanical.theta_rotor_rad - peer_position_rad);
        result.gap_max_rad = std::max(result.gap_max_rad, gap_rad);
        result.gap_final_rad = gap_rad;
        result.energy_final_j = static_cast<double>(status->passivity_energy_j);

        const double speed_rad_s = std::fabs(plant.mechanical().omega_rotor_rad_s);
        if (tick < early_end) {
            result.peak_early_rad_s = std::max(result.peak_early_rad_s, speed_rad_s);
        }
        if (tick >= late_start) {
            result.peak_late_rad_s = std::max(result.peak_late_rad_s, speed_rad_s);
        }

        if ((tick % config.log_every) == 0u) {
            std::fprintf(csv, "%llu,%.4f,%.9g,%.9g,%.9g,%.9g,%.6g,%.9g,%.9g,%.9g,%.6g,%.3f,%lld\n",
                         static_cast<unsigned long long>(tick),
                         static_cast<double>(tick + 1u) * tsim::kControlPeriodS * 1.0e3,
                         mechanical.theta_rotor_rad, peer_position_rad,
                         mechanical.omega_rotor_rad_s, status->torque_ref_nm, status->iq_a,
                         status->passivity_energy_j, status->passivity_excess_j,
                         status->passivity_peer_energy_j, status->passivity_damping_nm_s_per_rad,
                         (result.latency_count > 0u)
                             ? (result.latency_sum_us / static_cast<double>(result.latency_count))
                             : 0.0,
                         static_cast<long long>(result.skew_max_windows));
        }
    }

    std::fclose(csv);
    socket.close();

    const firmware_status_t *status = firmware_status();
    const double duration_s = static_cast<double>(config.ticks) * tsim::kControlPeriodS;
    /* Both numbers are in the same units and are not the same quantity. The pair
     * excess is what a two port controller damps on, and in two port mode it
     * already contains the far end's accumulator, so it must never be compared
     * against a one port run. The own port number is this drive's peak delivery
     * over the run, measured the same way in all three modes, and it is the one
     * the study tabulates. */
    const double generated_own_w = result.energy_max_j / duration_s;
    const double generated_pair_w = -result.excess_min_j / duration_s;
    const double growth =
        (result.peak_early_rad_s > 1.0e-6) ? result.peak_late_rad_s / result.peak_early_rad_s : 0.0;
    const double latency_avg_us =
        (result.latency_count > 0u)
            ? result.latency_sum_us / static_cast<double>(result.latency_count)
            : 0.0;

    std::printf("tandem node, %s side, port %u to %u\n", config.far ? "far" : "master",
                static_cast<unsigned>(bound_port), static_cast<unsigned>(config.peer_port));
    std::printf("  rendered       %.1f N m/rad, %.3f N m s/rad, controller %s\n",
                config.stiffness_nm_per_rad, config.damping_nm_s_per_rad,
                controller_name(config.controller));
    std::printf("  link           %.3f ms imposed, %.3f ms measured one way\n",
                static_cast<double>(config.delay_windows) * tsim::kControlPeriodS * 1.0e3,
                latency_avg_us * 1.0e-3);
    std::printf("  link packets   %llu delivered, %llu malformed, %llu peer timeouts\n",
                static_cast<unsigned long long>(result.delivered),
                static_cast<unsigned long long>(result.malformed),
                static_cast<unsigned long long>(result.peer_timeouts));
    std::printf("  own port       energy %.6g J minimum, %.6g J peak, %.6g J final\n",
                result.energy_min_j, result.energy_max_j, result.energy_final_j);
    std::printf("  pair           excess %.6g J minimum, generated %.4f W\n", result.excess_min_j,
                generated_pair_w);
    std::printf("  gap            %.6f rad peak, %.6f rad final\n", result.gap_max_rad,
                result.gap_final_rad);
    std::printf("  controller     damping %.3f N m s/rad peak, %llu ticks at the cap\n",
                result.damping_max_nm_s_per_rad, static_cast<unsigned long long>(result.saturated));
    std::printf("  speeds         early %.4f rad/s, late %.4f rad/s, growth %.4f\n",
                result.peak_early_rad_s, result.peak_late_rad_s, growth);
    std::printf("  final          angle %.6f rad, iq %.4f A, faults 0x%08x\n",
                plant.mechanical().theta_rotor_rad, status->iq_a, status->fault_bits);
    std::printf("  csv            %s\n", out_path.c_str());
    /* One line a launcher can read without guessing which of the paragraphs above
     * is the number it wants. */
    std::printf("SUMMARY role=%s delay_ms=%.3f stiffness=%.3f controller=%s growth=%.4f "
                "generated_own_w=%.4f generated_pair_w=%.4f energy_final_j=%.6g gap_max_rad=%.6f "
                "gap_final_rad=%.6f peak_late_rad_s=%.4f "
                "latency_avg_us=%.3f latency_max_us=%.3f delivered=%llu malformed=%llu "
                "timeouts=%llu skew_max=%lld saturated=%llu faults=%u\n",
                config.far ? "far" : "master",
                static_cast<double>(config.delay_windows) * tsim::kControlPeriodS * 1.0e3,
                config.stiffness_nm_per_rad, controller_name(config.controller), growth,
                generated_own_w, generated_pair_w, result.energy_final_j, result.gap_max_rad,
                result.gap_final_rad, result.peak_late_rad_s, latency_avg_us, result.latency_max_us,
                static_cast<unsigned long long>(result.delivered),
                static_cast<unsigned long long>(result.malformed),
                static_cast<unsigned long long>(result.peer_timeouts),
                static_cast<long long>(result.skew_max_windows),
                static_cast<unsigned long long>(result.saturated), status->fault_bits);
    return 0;
}
