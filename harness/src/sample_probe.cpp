/* Sampling instant probe. Opens the loop and looks inside one switching period.
 *
 * The firmware settles a current on a clamped rotor through the switching
 * plant, so the duty it produces is the duty a real drive would produce at that
 * operating point. The last command is then held, the plant is stepped one
 * microsecond at a time, and phase A is recorded for a whole period. The mean
 * of those fifty values is the number the ADC would have to return to be
 * unbiased, and the error at every offset within the period is the cost of
 * sampling anywhere else.
 *
 * No firmware runs during the capture, so nothing reacts to the sample and the
 * figure is a property of the plant and the modulator, not of the loop.
 */

#include <cstdio>
#include <exception>
#include <filesystem>
#include <string>

#include "firmware.h"
#include "hal.h"
#include "sim_hal.h"
#include "sim_loop.h"
#include "tsim/plant_params.h"
#include "tsim/switching_plant.h"

namespace {

constexpr std::uint32_t kSettleTicks = 400; /* 20 ms, well past the current loop */
/* 20 A, which is seven times the continuous rating and a third of stall. The
 * ripple in the winding is proportional to the voltage the modulator applies,
 * and at 2 A that voltage is 0.58 V, which leaves a 90 mA ripple and a figure
 * nobody can see. A stall test at 20 A is a real drive test, not a contrivance;
 * there is no thermal model yet to object, which is a limit this repository
 * states rather than hides. */
constexpr float kCurrentReferenceA = 20.0f;

} // namespace

int main(int argc, char **argv) {
    const char *out_path = "data/run_latest/day3_sample_instant.csv";
    if (argc > 2 && std::string(argv[1]) == "--out") {
        out_path = argv[2];
    }

    const tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    tsim::SwitchingPlant plant(params, tsim::inverter_params_realistic());
    /* Parked a quarter of an electrical turn in, so the q axis lines up with
     * phase A and the current that is being sampled is a real one, not a zero. */
    plant.reset(-tsim::kTwoPi / (4.0 * params.pole_pairs));
    plant.set_rotor_locked(true);
    sim_hal_attach(plant);
    sim_hal_set_board_dead_time_s(500.0e-9f);
    /* The sensing chain stays off: this figure is about where the sample is
     * taken, and ADC quantisation would add a floor under the thing being
     * measured. */
    sim_hal_set_imperfections(false);

    firmware_init();
    firmware_set_mode(FIRMWARE_MODE_CURRENT);
    firmware_set_current_reference_a(kCurrentReferenceA);
    for (std::uint32_t tick = 0; tick < kSettleTicks; ++tick) {
        sim_run_window(plant, tick);
    }

    /* Hold the last command and step exactly one switching period. The plant
     * has run a whole number of periods, so the carrier starts at its first
     * step and the sample offset in this capture is the same as in every window
     * before it. */
    tsim::InverterCommand command{};
    sim_hal_take_inverter_command(&command);
    plant.latch_inverter(command);

    const std::uint32_t steps = tsim::kStepsPerControl;
    double current_a[steps];
    double bus_v[steps];
    for (std::uint32_t step = 0; step < steps; ++step) {
        plant.step_one();
        const tsim::ElectricalReadout readout = plant.sample_electrical();
        current_a[step] = readout.ia_a;
        bus_v[step] = readout.bus_v;
    }

    double sum_a = 0.0;
    for (std::uint32_t step = 0; step < steps; ++step) {
        sum_a += current_a[step];
    }
    const double mean_a = sum_a / static_cast<double>(steps);

    double lowest_a = current_a[0];
    double highest_a = current_a[0];
    std::uint32_t lowest_step = 0u;
    std::uint32_t highest_step = 0u;
    for (std::uint32_t step = 0; step < steps; ++step) {
        if (current_a[step] < lowest_a) {
            lowest_a = current_a[step];
            lowest_step = step;
        }
        if (current_a[step] > highest_a) {
            highest_a = current_a[step];
            highest_step = step;
        }
    }

    try {
        const std::filesystem::path path(out_path);
        if (!path.parent_path().empty()) {
            std::filesystem::create_directories(path.parent_path());
        }
    } catch (const std::exception &error) {
        std::fprintf(stderr, "cannot create %s: %s\n", out_path, error.what());
        return 1;
    }

    std::FILE *csv = std::fopen(out_path, "w");
    if (csv == nullptr) {
        std::perror("fopen");
        return 1;
    }
    std::fprintf(csv, "step,offset_us,ia_a,error_a,bus_v\n");
    for (std::uint32_t step = 0; step < steps; ++step) {
        /* The sample is taken at the end of step index 24, which is 25 us. */
        const double offset_us = static_cast<double>(step + 1u) * 1.0;
        std::fprintf(csv, "%u,%.1f,%.9f,%.9f,%.9f\n", step, offset_us, current_a[step],
                     current_a[step] - mean_a, bus_v[step]);
    }
    std::fclose(csv);

    const double centre_a = current_a[tsim::kStepsPerControl / 2u - 1u];
    const double edge_a = current_a[steps - 1u];
    std::printf("sampling instant probe, switching plant, locked rotor\n");
    std::printf("  current reference  %.3f A\n", static_cast<double>(kCurrentReferenceA));
    std::printf("  window mean        %.9f A\n", mean_a);
    std::printf("  peak to peak       %.6f A\n", highest_a - lowest_a);
    std::printf("  maximum at         %u us, minimum at %u us\n", highest_step + 1u,
                lowest_step + 1u);
    std::printf("  sample at 25 us    %.9f A, error %+.6f A\n", centre_a, centre_a - mean_a);
    std::printf("  sample at 50 us    %.9f A, error %+.6f A\n", edge_a, edge_a - mean_a);
    std::printf("  bus at the centre  %.6f V\n", bus_v[tsim::kStepsPerControl / 2u - 1u]);
    std::printf("  csv                %s\n", out_path);
    return 0;
}
