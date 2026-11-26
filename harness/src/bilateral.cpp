/* Two joints and a link with delay, jitter and loss, with the passivity
 * observer on or off. The output is the stability boundary: how stiff a
 * coupling a given delay can carry.
 *
 * What is real here and what is not. The master joint is the switching plant
 * with the unmodified firmware running in impedance mode, so the spring it
 * renders is the spring this project has been measuring since day 4. The far
 * joint is the same mechanics with no drive: its bridge is held open and the
 * link's torque arrives as an external torque, clamped at the motor's stall
 * torque because that is the only property of the far drive this model has.
 * A real pair is two MCUs, and hal.h is a single MCU contract, so a second
 * firmware instance in this process is not available without changing the
 * header the firmware is built against. That is written down as the day's
 * largest cut rather than hidden behind a second controller in the harness.
 *
 * The interaction is the classic virtual coupling: each side renders a spring
 * to the other side's delayed position, so the pair is one spring of stiffness
 * k between two hands, with a delay in each direction. The operator on each
 * side is the same hand model day 4 used, because a pair of joints with nothing
 * holding them has no port to be passive at.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <vector>

#include "firmware.h"
#include "hal.h"
#include "link.h"
#include "sim_hal.h"
#include "sim_loop.h"
#include "tsim/ideal_plant.h"
#include "tsim/plant.h"
#include "tsim/plant_params.h"
#include "tsim/switching_plant.h"

namespace {

/* Stall torque of the motor the far side is pretending to have, N m. */
constexpr double kFarDriveLimitNm = 0.0461 * 61.8;
/* Encoder noise of the parameter table, rad RMS. The master commissions from
 * this, so a quiet encoder here would be a different (and flattering) joint. */
constexpr double kEncoderNoiseRad = 8.7266e-4;

struct StudyConfig {
    std::vector<double> stiffness_nm_per_rad{20.0, 40.0, 70.0, 100.0, 150.0, 220.0, 320.0};
    std::vector<double> delay_ms{0.5, 1.0, 2.0, 4.0, 8.0};
    /* Rendered damping, N m s/rad. Small on purpose: the impedance mode
     * multiplies it by the rotor speed, and a compliant shaft makes the rotor
     * move faster than the handle for the same handle motion, so 0.2 N m s/rad
     * on a 23 rad/s rotor release asks for 4.6 N m, twice stall. The first
     * version of this study railed its own current limit on the damping term
     * and measured the limit instead of the link. */
    double damping_nm_s_per_rad = 0.02;
    double jitter_ms = 1.0;
    double loss = 0.005;
    /* The initial stretch of the virtual coupling, rad. 2 mrad is chosen so that
     * even the stiffest coupling in the sweep asks for 0.64 N m, a quarter of
     * stall: the boundary is then a property of the loop and not of the current
     * limit, which is what the first version of this study measured. */
    double gap_rad = 0.002;
    /* The joint's own shaft, N m/rad. 250 is the steel shaft day 4 derived. A
     * large value is a rigid transmission, which is the case where the only
     * compliance in the loop is the rendered coupling and therefore the only
     * case where a boundary can be attributed to the link. */
    double shaft_stiffness_nm_per_rad = 250.0;
    std::uint64_t ticks = 10000;
    double trace_stiffness_nm_per_rad = 100.0;
    double trace_delay_ms = 2.0;
    bool trace_passivity = false;
    const char *sweep_out = "data/run_latest/day5_stability.csv";
    const char *trace_out = "";
};

struct RunResult {
    double growth = 0.0;
    double peak_early_rad_s = 0.0;
    double peak_late_rad_s = 0.0;
    double excess_min_j = 0.0;
    double damping_max_nm_s_per_rad = 0.0;
    double peak_iq_a = 0.0;
    bool faulted = false;
};

RunResult run_once(const StudyConfig &config, double stiffness_nm_per_rad, double delay_ms,
                   bool passivity, std::FILE *trace) {
    RunResult result;
    result.excess_min_j = 0.0;

    tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    params.mechanics = tsim::mechanics_params_hand();
    params.mechanics.coupling_stiffness_nm_per_rad = config.shaft_stiffness_nm_per_rad;

    tsim::SwitchingPlant master(params, tsim::inverter_params_realistic());
    master.reset(0.0);
    tsim::IdealPlant far(params);
    far.reset(config.gap_rad);

    sim_hal_attach(master);
    sim_hal_set_board_dead_time_s(5.0e-7f);
    sim_hal_set_encoder_noise_rad(static_cast<float>(kEncoderNoiseRad));
    sim_hal_set_imperfections(true);
    /* A board's converter runs before the bridge is enabled and the firmware
     * averages that first conversion for its current offsets. */
    sim_hal_latch_sample(master.sample_electrical(), 0u);

    firmware_init();
    firmware_set_passivity_control(passivity);
    firmware_set_impedance(static_cast<float>(stiffness_nm_per_rad),
                           static_cast<float>(config.damping_nm_s_per_rad));
    firmware_set_mode(FIRMWARE_MODE_IMPEDANCE);

    tsim::LinkConfig link_config;
    link_config.delay_windows =
        static_cast<std::uint32_t>(std::lround(delay_ms * 1.0e-3 / tsim::kControlPeriodS));
    link_config.jitter_windows =
        static_cast<std::uint32_t>(std::lround(config.jitter_ms * 1.0e-3 / tsim::kControlPeriodS));
    link_config.loss_probability = config.loss;
    link_config.seed = 0x5eed1234u;
    tsim::ControlLink link;
    link.configure(link_config);

    /* At t=0 the link has delivered nothing, and the honest initial state is the
     * true one: the far joint is at the gap and the master is at zero. Starting
     * both from zero would mean the first thing either side sees is a jump it
     * cannot tell from a packet. */
    tsim::LinkSample far_sample;
    far_sample.position_rad = config.gap_rad;
    far_sample.valid = true;
    tsim::LinkSample master_sample;
    master_sample.position_rad = 0.0;
    master_sample.valid = true;

    const std::uint64_t early_end = std::max<std::uint64_t>(config.ticks / 10u, 1u);
    const std::uint64_t late_start = config.ticks - config.ticks / 10u;
    std::uint64_t sequence = 0u;

    for (std::uint64_t tick = 0u; tick < config.ticks; ++tick) {
        const tsim::MechanicalState master_state = master.mechanical();
        const tsim::MechanicalState far_state = far.mechanical();

        tsim::LinkSample to_far;
        to_far.position_rad = master_state.theta_rotor_rad;
        to_far.velocity_rad_s = master_state.omega_rotor_rad_s;
        to_far.sequence = sequence;
        sequence += 1u;
        to_far.valid = true;
        link.send_to_far(to_far, tick);

        tsim::LinkSample to_master;
        to_master.position_rad = far_state.theta_rotor_rad;
        to_master.velocity_rad_s = far_state.omega_rotor_rad_s;
        to_master.sequence = sequence;
        sequence += 1u;
        to_master.valid = true;
        link.send_to_master(to_master, tick);

        const tsim::LinkSample received_at_master = link.receive_at_master(tick);
        if (received_at_master.valid) {
            far_sample = received_at_master;
        }
        const tsim::LinkSample received_at_far = link.receive_at_far(tick);
        if (received_at_far.valid) {
            master_sample = received_at_far;
        }

        /* The master's render: a spring to where the far handle was, which is
         * the firmware's own impedance mode and not a controller written here. */
        firmware_set_position_reference_rad(static_cast<float>(far_sample.position_rad));

        double far_torque_nm =
            stiffness_nm_per_rad * (master_sample.position_rad - far_state.theta_rotor_rad) -
            config.damping_nm_s_per_rad * far_state.omega_rotor_rad_s;
        if (far_torque_nm > kFarDriveLimitNm) {
            far_torque_nm = kFarDriveLimitNm;
        }
        if (far_torque_nm < -kFarDriveLimitNm) {
            far_torque_nm = -kFarDriveLimitNm;
        }
        /* External torque opposes positive rotation, so the drive torque goes in
         * with the opposite sign. */
        far.set_external_torque_nm(-far_torque_nm);

        sim_run_window(master, tick);
        sim_run_remote_window(far);

        const double speed_magnitude = std::fabs(master.mechanical().omega_rotor_rad_s);
        if (tick < early_end) {
            result.peak_early_rad_s = std::max(result.peak_early_rad_s, speed_magnitude);
        }
        if (tick >= late_start) {
            result.peak_late_rad_s = std::max(result.peak_late_rad_s, speed_magnitude);
        }

        const firmware_status_t *status = firmware_status();
        result.excess_min_j =
            std::min(result.excess_min_j, static_cast<double>(status->passivity_excess_j));
        result.damping_max_nm_s_per_rad =
            std::max(result.damping_max_nm_s_per_rad,
                     static_cast<double>(status->passivity_damping_nm_s_per_rad));
        result.peak_iq_a =
            std::max(result.peak_iq_a, std::fabs(static_cast<double>(status->iq_ref_a)));
        if (status->fault_bits != 0u) {
            result.faulted = true;
        }

        if (trace != nullptr && (tick % 5u) == 0u) {
            std::fprintf(trace, "%llu,%.4f,%.9g,%.9g,%.9g,%.9g,%.9g,%.6g,%.6g,%.9g,%.6g,%.6g\n",
                         static_cast<unsigned long long>(tick),
                         static_cast<double>(tick + 1u) * tsim::kControlPeriodS * 1.0e3,
                         master.mechanical().theta_rotor_rad, far.mechanical().theta_rotor_rad,
                         far_sample.position_rad, master.mechanical().omega_rotor_rad_s,
                         far.mechanical().omega_rotor_rad_s, status->torque_ref_nm, far_torque_nm,
                         status->passivity_excess_j, status->passivity_damping_nm_s_per_rad,
                         status->iq_a);
        }
    }

    /* The growth of the oscillation over the run. Below one the pair is settling,
     * above one it is building, and that sign is the boundary. The early window
     * is the release of the initial gap and the late one is the last 10 percent
     * of the run, far enough apart that a decaying ring has died and a growing
     * one has not. */
    result.growth =
        (result.peak_early_rad_s > 1.0e-6) ? result.peak_late_rad_s / result.peak_early_rad_s : 0.0;

    if (trace != nullptr) {
        std::printf("  link           %.3f ms, jitter %.3f ms, loss %.4f\n", delay_ms,
                    config.jitter_ms, config.loss);
        std::printf("  coupling       %.1f N m/rad, damping %.3f N m s/rad\n", stiffness_nm_per_rad,
                    config.damping_nm_s_per_rad);
        std::printf("  passivity      %s\n", passivity ? "on" : "off");
        std::printf("  commissioned   speed crossover %.1f rad/s\n",
                    firmware_status()->speed_crossover_rad_s);
        std::printf("  growth         %.4f (early %.3f rad/s, late %.3f rad/s)\n", result.growth,
                    result.peak_early_rad_s, result.peak_late_rad_s);
        std::printf("  po excess      %.6g J minimum, damping %.3f N m s/rad peak\n",
                    result.excess_min_j, result.damping_max_nm_s_per_rad);
        std::printf("  peak iq        %.3f A, faults %s\n", result.peak_iq_a,
                    result.faulted ? "latched" : "none");
    }
    return result;
}

std::vector<double> parse_list(const char *text) {
    std::vector<double> values;
    const char *cursor = text;
    while (*cursor != '\0') {
        char *end = nullptr;
        const double value = std::strtod(cursor, &end);
        if (end == cursor) {
            break;
        }
        values.push_back(value);
        cursor = end;
        while (*cursor == ',' || *cursor == ' ') {
            ++cursor;
        }
    }
    return values;
}

bool read_on_off(const char *text, bool *out_value) {
    if (std::strcmp(text, "on") == 0) {
        *out_value = true;
        return true;
    }
    if (std::strcmp(text, "off") == 0) {
        *out_value = false;
        return true;
    }
    return false;
}

void print_usage(const char *program) {
    std::fprintf(stderr,
                 "usage: %s [--stiffness-list X,X,...] [--delay-list X,X,...] [--damping X]\n"
                 "          [--jitter-ms X] [--loss X] [--gap-rad X] [--shaft-stiffness X] "
                 "[--ticks N]\n"
                 "          [--out PATH] [--trace] [--trace-stiffness X] [--trace-delay-ms X]\n"
                 "          [--trace-passivity on|off]\n",
                 program);
}

} // namespace

int main(int argc, char **argv) {
    StudyConfig config;
    bool want_trace = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--stiffness-list") == 0 && i + 1 < argc) {
            config.stiffness_nm_per_rad = parse_list(argv[++i]);
        } else if (std::strcmp(argv[i], "--delay-list") == 0 && i + 1 < argc) {
            config.delay_ms = parse_list(argv[++i]);
        } else if (std::strcmp(argv[i], "--damping") == 0 && i + 1 < argc) {
            config.damping_nm_s_per_rad = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(argv[i], "--jitter-ms") == 0 && i + 1 < argc) {
            config.jitter_ms = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(argv[i], "--loss") == 0 && i + 1 < argc) {
            config.loss = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(argv[i], "--shaft-stiffness") == 0 && i + 1 < argc) {
            config.shaft_stiffness_nm_per_rad = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(argv[i], "--gap-rad") == 0 && i + 1 < argc) {
            config.gap_rad = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(argv[i], "--ticks") == 0 && i + 1 < argc) {
            config.ticks = std::strtoull(argv[++i], nullptr, 10);
        } else if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            config.sweep_out = argv[++i];
        } else if (std::strcmp(argv[i], "--trace") == 0) {
            want_trace = true;
            config.trace_out = "data/run_latest/day5_bilateral_trace.csv";
        } else if (std::strcmp(argv[i], "--trace-stiffness") == 0 && i + 1 < argc) {
            config.trace_stiffness_nm_per_rad = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(argv[i], "--trace-delay-ms") == 0 && i + 1 < argc) {
            config.trace_delay_ms = std::strtod(argv[++i], nullptr);
        } else if (std::strcmp(argv[i], "--trace-passivity") == 0 && i + 1 < argc) {
            if (!read_on_off(argv[++i], &config.trace_passivity)) {
                std::fprintf(stderr, "trace-passivity wants on or off\n");
                return 2;
            }
        } else if (std::strcmp(argv[i], "--trace-out") == 0 && i + 1 < argc) {
            config.trace_out = argv[++i];
            want_trace = true;
        } else if (std::strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            print_usage(argv[0]);
            return 2;
        }
    }
    if (config.ticks == 0u) {
        std::fprintf(stderr, "--ticks wants a nonzero count\n");
        return 2;
    }

    const std::filesystem::path sweep_path(config.sweep_out);
    try {
        if (!sweep_path.parent_path().empty()) {
            std::filesystem::create_directories(sweep_path.parent_path());
        }
    } catch (const std::exception &error) {
        std::fprintf(stderr, "cannot create %s: %s\n", sweep_path.string().c_str(), error.what());
        return 1;
    }

    std::FILE *csv = std::fopen(config.sweep_out, "w");
    if (csv == nullptr) {
        std::perror("fopen");
        return 1;
    }
    std::fprintf(csv, "delay_ms,stiffness_nm_per_rad,passivity,growth,peak_early_rad_s,"
                      "peak_late_rad_s,excess_min_j,damping_max_nm_s_per_rad,peak_iq_a,faulted\n");

    std::printf("two joints over a delayed link, switching plant on the master side\n");
    std::printf("  shaft          %.0f N m/rad, gap %.1f mrad, damping %.3f N m s/rad\n",
                config.shaft_stiffness_nm_per_rad, config.gap_rad * 1.0e3,
                config.damping_nm_s_per_rad);
    for (const double delay_ms : config.delay_ms) {
        for (const double stiffness : config.stiffness_nm_per_rad) {
            for (const bool passivity : {false, true}) {
                const RunResult result = run_once(config, stiffness, delay_ms, passivity, nullptr);
                std::fprintf(csv, "%.4f,%.3f,%d,%.4f,%.6g,%.6g,%.6g,%.6g,%.6g,%d\n", delay_ms,
                             stiffness, passivity ? 1 : 0, result.growth, result.peak_early_rad_s,
                             result.peak_late_rad_s, result.excess_min_j,
                             result.damping_max_nm_s_per_rad, result.peak_iq_a,
                             result.faulted ? 1 : 0);
                std::printf("  delay %5.2f ms  k %6.1f N m/rad  passivity %-3s  growth %7.3f  "
                            "excess %9.3g J  beta %6.2f\n",
                            delay_ms, stiffness, passivity ? "on" : "off", result.growth,
                            result.excess_min_j, result.damping_max_nm_s_per_rad);
            }
        }
    }
    std::fclose(csv);
    std::printf("  wrote %s\n", config.sweep_out);

    if (want_trace) {
        const std::filesystem::path trace_path(config.trace_out);
        if (!trace_path.parent_path().empty()) {
            std::filesystem::create_directories(trace_path.parent_path());
        }
        std::FILE *trace = std::fopen(config.trace_out, "w");
        if (trace == nullptr) {
            std::perror("fopen");
            return 1;
        }
        std::printf("trace run\n");
        std::fprintf(trace, "tick,time_ms,theta_master_rad,theta_far_rad,theta_ref_rad,"
                            "omega_master_rad_s,omega_far_rad_s,torque_master_nm,torque_far_nm,"
                            "excess_j,damping_nm_s_per_rad,iq_a\n");
        (void)run_once(config, config.trace_stiffness_nm_per_rad, config.trace_delay_ms,
                       config.trace_passivity, trace);
        std::fclose(trace);
        std::printf("  wrote %s\n", config.trace_out);
    }
    return 0;
}
