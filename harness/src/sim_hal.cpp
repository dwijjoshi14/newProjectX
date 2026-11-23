/* Implementation of hal.h on the host. Firmware calls these through the header
 * only. Single threaded in deterministic mode; the real time mode is added
 * later and will need atomics on the sample exchange. */

#include "sim_hal.h"

#include <cmath>

#include "hal.h"
#include "tsim/plant.h"

namespace {

struct SimHalState {
    tsim::IPlant *plant = nullptr;
    hal_board_config_t board{};
    float duty[3] = {0.5f, 0.5f, 0.5f};
    bool enabled = false;
    uint32_t dead_time_ns = 500u;
    tsim::ElectricalReadout latched{};
    bool latched_valid = false;
    uint32_t latched_timestamp_us = 0u;
    uint32_t fault_bits = 0u;
    uint64_t time_us = 0u;
    uint32_t telemetry_drops = 0u;
    hal_tick_fn_t tick_fn = nullptr;
    bool timing_running = false;
};

/* One simulation, one HAL instance. The firmware is single instance too. */
SimHalState s_state;

} // namespace

void sim_hal_attach(tsim::IPlant &plant) {
    s_state.plant = &plant;
    /* A new plant means a new run. Clearing the latch matters: the firmware
     * averages the latched sample at boot to find its current offsets, and a
     * sample left over from an earlier run would be averaged as if it belonged
     * to this one. A converter that has not converted yet has no sample. */
    s_state.latched = tsim::ElectricalReadout{};
    s_state.latched_valid = false;
    s_state.latched_timestamp_us = 0u;
    s_state.duty[0] = 0.5f;
    s_state.duty[1] = 0.5f;
    s_state.duty[2] = 0.5f;
    s_state.enabled = false;
    s_state.fault_bits = 0u;
    const tsim::PlantParams &params = plant.params();
    s_state.board.pole_pairs = static_cast<uint32_t>(params.pole_pairs);
    s_state.board.encoder_counts_per_rev = 16384; /* 14 bit, docs/parameters.md */
    s_state.board.encoder_zero_offset_counts = 0;
    s_state.board.encoder_inverted = false;
    s_state.board.shunt_resistance_ohm = 5.0e-4f;   /* ASSUMED, docs/parameters.md */
    s_state.board.current_amp_gain_v_per_v = 50.0f; /* ASSUMED, docs/parameters.md */
    s_state.board.adc_reference_v = 3.3f;           /* ASSUMED, docs/parameters.md */
    s_state.board.adc_full_scale_counts = 4095u;    /* 12 bit, docs/parameters.md */
    s_state.board.bus_voltage_nominal_v = static_cast<float>(params.bus_voltage_v);
    s_state.board.dead_time_s = 5.0e-7f;  /* 500 ns, ASSUMED */
    s_state.board.pwm_period_s = 5.0e-5f; /* 1 / 20 kHz, s */
}

void sim_hal_take_inverter_command(tsim::InverterCommand *out_command) {
    out_command->duty_a = s_state.duty[0];
    out_command->duty_b = s_state.duty[1];
    out_command->duty_c = s_state.duty[2];
    out_command->bridge_enabled = s_state.enabled;
}

void sim_hal_latch_sample(const tsim::ElectricalReadout &sample, std::uint32_t timestamp_us) {
    s_state.latched = sample;
    s_state.latched_valid = true;
    s_state.latched_timestamp_us = timestamp_us;
}

void sim_hal_set_time_us(std::uint64_t time_us) {
    s_state.time_us = time_us;
}

std::uint32_t sim_hal_telemetry_drops() {
    return s_state.telemetry_drops;
}

extern "C" {

hal_status_t hal_init(void) {
    if (s_state.plant == nullptr) {
        return HAL_ERR_STATE;
    }
    s_state.enabled = false;
    s_state.fault_bits = 0u;
    /* No conversion has completed since init, so the boot offset calibration
     * has nothing to average and leaves the offsets at zero. On a board the
     * ADC free runs and this returns real samples. */
    s_state.latched_valid = false;
    return HAL_OK;
}

const hal_board_config_t *hal_board_config(void) {
    return &s_state.board;
}

hal_status_t hal_pwm_set_duty(float duty_a, float duty_b, float duty_c) {
    const float requested[3] = {duty_a, duty_b, duty_c};
    /* Validate all three before writing any. Writing as we went meant a vector
     * with one bad entry left two phases on the new duty and one on the old,
     * which is not a state any real timer can reach. */
    for (uint32_t i = 0u; i < 3u; ++i) {
        if (!(requested[i] >= 0.0f && requested[i] <= 1.0f)) {
            return HAL_ERR_ARGUMENT;
        }
    }
    for (uint32_t i = 0u; i < 3u; ++i) {
        s_state.duty[i] = requested[i];
    }
    return HAL_OK;
}

void hal_pwm_enable(bool enable) {
    s_state.enabled = enable;
}

hal_status_t hal_pwm_set_dead_time_ns(uint32_t dead_time_ns) {
    if (dead_time_ns > 2000u) {
        return HAL_ERR_ARGUMENT;
    }
    s_state.dead_time_ns = dead_time_ns;
    return HAL_OK;
}

hal_status_t hal_current_read(hal_current_sample_t *out) {
    if (out == nullptr) {
        return HAL_ERR_ARGUMENT;
    }
    out->ia_a = static_cast<float>(s_state.latched.ia_a);
    out->ib_a = static_cast<float>(s_state.latched.ib_a);
    out->ic_a = static_cast<float>(s_state.latched.ic_a);
    out->timestamp_us = s_state.latched_timestamp_us;
    out->valid = s_state.latched_valid;
    return HAL_OK;
}

hal_status_t hal_encoder_read(hal_encoder_sample_t *out) {
    if (out == nullptr || s_state.plant == nullptr) {
        return HAL_ERR_ARGUMENT;
    }
    const double theta_rotor_rad = s_state.plant->mechanical().theta_rotor_rad;
    const double counts_per_rad =
        static_cast<double>(s_state.board.encoder_counts_per_rev) / tsim::kTwoPi;
    out->position_counts =
        static_cast<std::int64_t>(std::llround(theta_rotor_rad * counts_per_rad));
    out->timestamp_us = static_cast<std::uint32_t>(s_state.time_us);
    out->index_seen = true;
    out->valid = true;
    return HAL_OK;
}

float hal_bus_voltage_read_v(void) {
    return static_cast<float>(s_state.latched.bus_v);
}

float hal_temperature_read_c(void) {
    return 25.0f; /* fixed ambient, thermal network arrives in rung 6 */
}

uint32_t hal_fault_read(void) {
    return s_state.fault_bits;
}

void hal_fault_clear(void) {
    s_state.fault_bits = 0u;
}

void hal_fault_latch(uint32_t code) {
    s_state.fault_bits |= code;
}

void hal_watchdog_kick(void) {}

hal_status_t hal_watchdog_set_timeout_us(uint32_t timeout_us) {
    (void)timeout_us;
    return HAL_OK;
}

uint64_t hal_time_us(void) {
    return s_state.time_us;
}

uint32_t hal_cpu_cycles(void) {
    return static_cast<uint32_t>(s_state.time_us);
}

void hal_delay_us(uint32_t delay_us) {
    s_state.time_us += delay_us;
}

hal_status_t hal_set_control_tick(hal_tick_fn_t tick_fn) {
    if (tick_fn == nullptr) {
        return HAL_ERR_ARGUMENT;
    }
    s_state.tick_fn = tick_fn;
    return HAL_OK;
}

hal_status_t hal_start_control_timing(void) {
    /* The harness drives the tick, so this must not start a second clock. */
    return HAL_ERR_STATE;
}

void hal_stop_control_timing(void) {
    s_state.timing_running = false;
}

uint32_t hal_irq_save(void) {
    return 0u;
}

void hal_irq_restore(uint32_t mask) {
    (void)mask;
}

bool hal_telemetry_push(const void *frame, size_t length_bytes) {
    (void)frame;
    (void)length_bytes;
    s_state.telemetry_drops += 1u;
    return false;
}

uint32_t hal_telemetry_drop_count(void) {
    return s_state.telemetry_drops;
}

void hal_status_led_set(bool on) {
    (void)on;
}

void hal_panic(uint32_t code, uint32_t line) {
    (void)code;
    (void)line;
    for (;;) {
    }
}

} // extern "C"
