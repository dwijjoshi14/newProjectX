#include "sim_loop.h"

#include "firmware.h"
#include "sim_hal.h"
#include "tsim/plant.h"

void sim_run_remote_window(tsim::IPlant &plant) {
    tsim::InverterCommand command{};
    command.duty_a = 0.5f;
    command.duty_b = 0.5f;
    command.duty_c = 0.5f;
    command.bridge_enabled = false;
    plant.latch_inverter(command);
    for (std::uint32_t i = 0u; i < tsim::kStepsPerControl; ++i) {
        plant.step_one();
    }
}

void sim_run_window(tsim::IPlant &plant, std::uint64_t tick) {
    tsim::InverterCommand command{};
    sim_hal_take_inverter_command(&command);
    plant.latch_inverter(command);

    const std::uint32_t half_window = tsim::kStepsPerControl / 2u;
    for (std::uint32_t i = 0u; i < half_window; ++i) {
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
}
