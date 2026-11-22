/* sim_hal.h, the harness side of hal.h. The harness owns the clock and tells
 * the HAL when a window starts and when the ADC latched. */
#ifndef TSIM_HARNESS_SIM_HAL_H
#define TSIM_HARNESS_SIM_HAL_H

#include <cstdint>

#include "tsim/plant.h"

namespace tsim {
class IPlant;
}

/* Bind the HAL to a plant. Must be called before firmware_init. */
void sim_hal_attach(tsim::IPlant &plant);

/* Duty written by the last firmware tick, to be applied at the next window
 * start, phase and inverter frames. */
void sim_hal_take_inverter_command(tsim::InverterCommand *out_command);

/* Publish the sample the converter latched, phase and inverter frames. */
void sim_hal_latch_sample(const tsim::ElectricalReadout &sample, std::uint32_t timestamp_us);

/* Advance the HAL clock, us. The harness calls this, nothing else. */
void sim_hal_set_time_us(std::uint64_t time_us);

/* Telemetry frames refused by the ring buffer, count. */
std::uint32_t sim_hal_telemetry_drops();

#endif /* TSIM_HARNESS_SIM_HAL_H */
