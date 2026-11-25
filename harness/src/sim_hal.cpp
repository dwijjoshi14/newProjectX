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

    /* Encoder noise, rad RMS. ASSUMED at 0.05 deg in docs/parameters.md, which
     * at 14 bits is 2.3 counts: the velocity estimate is a ten tick difference,
     * so it arrives with 2.5 rad/s of noise on it. */
    float encoder_noise_rad = 8.7266e-4f;

    /* Sensing chain, all off until sim_hal_set_imperfections(true). */
    bool imperfections = false;
    uint32_t rng = 0x1f123bb5u;
    /* Per phase shunt gain error and amplifier offset from the mid scale
     * bias. ASSUMED in docs/parameters.md, deterministic so a rerun matches. */
    float gain_error[3] = {1.01f, 0.992f, 1.004f};
    float offset_v[3] = {1.5e-3f, -2.0e-3f, 0.8e-3f};
};

/* One simulation, one HAL instance. The firmware is single instance too. */
SimHalState s_state;

/* xorshift32. Deterministic, in repo, and seeded so that a repeated run of the
 * same binary sees the same noise. */
uint32_t sim_rng_next(void) {
    uint32_t x = s_state.rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_state.rng = x;
    return x;
}

/* Uniform on [-0.5, 0.5). */
float sim_uniform(void) {
    return static_cast<float>(static_cast<std::int32_t>(sim_rng_next())) / 4294967296.0f;
}

/* Three uniforms summed, scaled to unit variance. Twice the sigma of one
 * uniform is 0.5, so two is the factor that puts this at one. Good enough for
 * encoder noise, and it needs no library. */
float sim_gaussian(void) {
    return (sim_uniform() + sim_uniform() + sim_uniform()) * 2.0f;
}

float sim_shunt_volts_per_amp(void) {
    return s_state.board.shunt_resistance_ohm * s_state.board.current_amp_gain_v_per_v;
}

float sim_adc_lsb_volts(void) {
    return s_state.board.adc_reference_v / static_cast<float>(s_state.board.adc_full_scale_counts);
}

/* True phase current through the shunt amplifier and the converter, A. The
 * amplifier is biased at mid scale so that both current directions fit inside
 * the converter's range; the firmware removes that bias at boot, which is what
 * makes the boot calibration worth having. */
float sim_measure_current(float true_a, uint32_t phase) {
    if (!s_state.imperfections) {
        return true_a;
    }
    const float bias_v = 0.5f * s_state.board.adc_reference_v;
    const float volts = bias_v + true_a * sim_shunt_volts_per_amp() * s_state.gain_error[phase] +
                        s_state.offset_v[phase];
    float clamped_v = volts;
    if (clamped_v < 0.0f) {
        clamped_v = 0.0f;
    }
    if (clamped_v > s_state.board.adc_reference_v) {
        clamped_v = s_state.board.adc_reference_v;
    }
    const float counts = std::nearbyint(clamped_v / sim_adc_lsb_volts());
    return (counts * sim_adc_lsb_volts() - bias_v) / sim_shunt_volts_per_amp();
}

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
    /* The drive is commissioned against the motor it drives, so the decoupling
     * and feedforward terms use the same constants the plant does. */
    s_state.board.motor_inductance_h = static_cast<float>(params.inductance_phase_h);
    s_state.board.motor_flux_linkage_wb = static_cast<float>(params.flux_linkage_wb);
    /* Commissioned against the joint it drives, which includes what hangs off
     * the coupling. A compliant coupling only reflects the load below its own
     * resonance, which is the band the speed loop is designed in. */
    s_state.board.motor_inertia_kg_m2 = static_cast<float>(params.inertia_kg_m2);
    s_state.board.reflected_load_inertia_kg_m2 =
        static_cast<float>(params.mechanics.load_inertia_kg_m2);
    s_state.board.coupling_stiffness_nm_per_rad =
        static_cast<float>(params.mechanics.coupling_stiffness_nm_per_rad);
    /* Off until the chain is switched on, so a board that is never told about
     * its sensor does not commission for one. */
    s_state.board.encoder_noise_rad = 0.0f;
}

void sim_hal_set_imperfections(bool enable) {
    s_state.imperfections = enable;
    s_state.rng = 0x1f123bb5u;
    /* A chain that is off has no noise, and the drive commissions from the
     * number it is told. Leaving the noise in the board config while the plant
     * is clean would make the drive derate for a sensor it does not have. */
    s_state.board.encoder_noise_rad = enable ? s_state.encoder_noise_rad : 0.0f;
}

void sim_hal_set_encoder_noise_rad(float noise_rad) {
    s_state.encoder_noise_rad = (noise_rad > 0.0f) ? noise_rad : 0.0f;
    s_state.board.encoder_noise_rad = s_state.encoder_noise_rad;
}

void sim_hal_set_board_dead_time_s(float dead_time_s) {
    s_state.board.dead_time_s = dead_time_s;
    s_state.dead_time_ns = static_cast<uint32_t>(std::lround(dead_time_s * 1.0e9f));
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
    /* The latch is left alone on purpose. On a board the ADC is free running
     * before the bridge is enabled, so the boot offset calibration averages
     * real conversions. sim_hal_attach clears the latch for a fresh run, so the
     * harness still starts with no sample and calibrates to zero offsets. */
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
    out->ia_a = sim_measure_current(static_cast<float>(s_state.latched.ia_a), 0u);
    out->ib_a = sim_measure_current(static_cast<float>(s_state.latched.ib_a), 1u);
    out->ic_a = sim_measure_current(static_cast<float>(s_state.latched.ic_a), 2u);
    out->timestamp_us = s_state.latched_timestamp_us;
    out->valid = s_state.latched_valid;
    return HAL_OK;
}

hal_status_t hal_encoder_read(hal_encoder_sample_t *out) {
    if (out == nullptr || s_state.plant == nullptr) {
        return HAL_ERR_ARGUMENT;
    }
    const tsim::MechanicalState mechanical = s_state.plant->mechanical();
    const double counts_per_rad =
        static_cast<double>(s_state.board.encoder_counts_per_rev) / tsim::kTwoPi;
    double theta_rotor_rad = mechanical.theta_rotor_rad;
    double counts = theta_rotor_rad * counts_per_rad;
    if (s_state.imperfections) {
        /* Stated 1.5 us of decoder latency, applied as a shift of the angle,
         * which is exact at constant speed. The noise is 0.05 deg RMS, which
         * at 14 bits is 2.3 counts, so the encoder is not quiet enough to be
         * treated as exact between samples. */
        theta_rotor_rad -= mechanical.omega_rotor_rad_s * 1.5e-6;
        counts = theta_rotor_rad * counts_per_rad +
                 static_cast<double>(s_state.encoder_noise_rad * static_cast<float>(counts_per_rad)) *
                     static_cast<double>(sim_gaussian());
    }
    out->position_counts = static_cast<std::int64_t>(std::llround(counts));
    out->timestamp_us = static_cast<std::uint32_t>(s_state.time_us);
    out->index_seen = true;
    out->valid = true;
    return HAL_OK;
}

float hal_bus_voltage_read_v(void) {
    const float bus_v = static_cast<float>(s_state.latched.bus_v);
    if (!s_state.imperfections) {
        return bus_v;
    }
    /* A bus divider that puts 40 V across a 12 bit converter is 9.8 mV per
     * count; 10 mV is the same thing rounded to a number a person can read. */
    const float span_v = 40.0f;
    const float lsb_v = span_v / static_cast<float>(s_state.board.adc_full_scale_counts);
    return std::nearbyint(bus_v / lsb_v) * lsb_v;
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
