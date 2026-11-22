/* Deterministic harness. One loop owns time: it applies the duty the firmware
 * wrote last tick, integrates half a window to the ADC trigger, latches the
 * sample, integrates the rest, then calls the firmware. Sample to actuation is
 * therefore 25 us to 75 us, which is the real pipeline delay and not an
 * artefact of the harness. */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>

#include "firmware.h"
#include "sim_hal.h"
#include "tsim/ideal_plant.h"
#include "tsim/plant.h"
#include "tsim/plant_params.h"

namespace {

struct Options {
    std::uint64_t ticks = 2000; /* 100 ms at 20 kHz */
    std::string out_path = "data/run_latest/timeseries.csv";
    std::uint64_t log_every = 10; /* ticks between CSV rows */
};

Options parse_options(int argc, char **argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--ticks") == 0 && i + 1 < argc) {
            options.ticks = std::strtoull(argv[++i], nullptr, 10);
        } else if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            options.out_path = argv[++i];
        } else {
            std::fprintf(stderr, "usage: %s [--ticks N] [--out PATH]\n", argv[0]);
            std::exit(2);
        }
    }
    if (options.log_every == 0) {
        options.log_every = 1;
    }
    return options;
}

} // namespace

int main(int argc, char **argv) {
    const Options options = parse_options(argc, argv);

    tsim::IdealPlant plant(tsim::maxon_ec_i_40_70w_params());
    plant.reset(0.0);
    sim_hal_attach(plant);

    firmware_init();
    if (firmware_status()->fault_bits != 0u) {
        std::fprintf(stderr, "firmware latched a fault during boot\n");
        return 1;
    }

    const std::filesystem::path out_path(options.out_path);
    if (!out_path.parent_path().empty()) {
        std::filesystem::create_directories(out_path.parent_path());
    }
    std::FILE *csv = std::fopen(options.out_path.c_str(), "w");
    if (csv == nullptr) {
        std::perror("fopen");
        return 1;
    }
    std::fprintf(csv, "tick,time_us,theta_m_rad,omega_rad_s,ia_a,duty_a\n");

    for (std::uint64_t tick = 0; tick < options.ticks; ++tick) {
        tsim::InverterCommand command{};
        sim_hal_take_inverter_command(&command);
        plant.latch_inverter(command);

        const std::uint32_t half_window = tsim::kStepsPerControl / 2u;
        for (std::uint32_t i = 0; i < half_window; ++i) {
            plant.step_one();
        }

        const std::uint64_t sample_time_ns = tick * tsim::kControlPeriodNs + tsim::kSampleOffsetNs;
        sim_hal_latch_sample(plant.sample_electrical(),
                             static_cast<std::uint32_t>(sample_time_ns / 1000u));

        for (std::uint32_t i = half_window; i < tsim::kStepsPerControl; ++i) {
            plant.step_one();
        }

        sim_hal_set_time_us((tick + 1u) * tsim::kControlPeriodNs / 1000u);
        firmware_tick();

        if (tick % options.log_every == 0) {
            const tsim::MechanicalState mechanical = plant.mechanical();
            const tsim::ElectricalReadout electrical = plant.sample_electrical();
            std::fprintf(csv, "%llu,%.3f,%.9f,%.9f,%.9f,%.6f\n",
                         static_cast<unsigned long long>(tick),
                         static_cast<double>(tick) * tsim::kControlPeriodS * 1.0e6,
                         mechanical.theta_rotor_rad, mechanical.omega_rotor_rad_s, electrical.ia_a,
                         static_cast<double>(command.duty_a));
        }
    }

    std::fclose(csv);

    const tsim::MechanicalState mechanical = plant.mechanical();
    const tsim::ElectricalReadout electrical = plant.sample_electrical();
    std::printf("deterministic run\n");
    std::printf("  ticks            %llu (%.3f s)\n",
                static_cast<unsigned long long>(plant.step_count() / tsim::kStepsPerControl),
                plant.time_s());
    std::printf("  rotor angle      %.6f rad\n", mechanical.theta_rotor_rad);
    std::printf("  rotor speed      %.6f rad/s\n", mechanical.omega_rotor_rad_s);
    std::printf("  phase A current  %.6f A\n", electrical.ia_a);
    std::printf("  firmware ticks   %u\n", firmware_status()->ticks);
    std::printf("  faults           0x%08x\n", firmware_status()->fault_bits);
    std::printf("  telemetry drops  %u\n", sim_hal_telemetry_drops());
    std::printf("  csv              %s\n", options.out_path.c_str());
    return 0;
}
