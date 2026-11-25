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

/* Turn the sensing chain on: shunt gain error, amplifier offset, 12 bit
 * quantisation, 1.5 us of encoder latency and 0.05 deg RMS of encoder noise.
 * Off by default, so a test that wants to measure the controller measures the
 * controller and not the converter. Every value is ASSUMED in
 * docs/parameters.md. Resets the noise seed when called, so two runs of the
 * same binary see the same noise. */
void sim_hal_set_imperfections(bool enable);

/* Gate dead time the board config reports, s. The harness sets this to the
 * inverter model's dead time, because a drive can only compensate what its
 * config claims, and a study that changes one without the other is measuring
 * the mismatch rather than the compensation. */
void sim_hal_set_board_dead_time_s(float dead_time_s);

/* Encoder noise, rad RMS, as measured and as reported to the drive. The plant
 * and the board config are set together, because the drive commissions its own
 * bandwidth from the number it is given and a study that changes only the plant
 * is measuring the mismatch. Default is the 0.05 deg of docs/parameters.md, and
 * zero turns the noise off without touching the rest of the sensing chain. */
void sim_hal_set_encoder_noise_rad(float noise_rad);

/* Telemetry frames refused by the ring buffer, count. */
std::uint32_t sim_hal_telemetry_drops();

#endif /* TSIM_HARNESS_SIM_HAL_H */
