/* Stub implementation of hal.h for the compile only portable target and for
 * firmware tests that do not need a plant. Every call succeeds and reports
 * zero physics. Values here match docs/parameters.md, they are not measured. */

#include "hal.h"

#include "stub_hal_probe.h"

#define STUB_BUS_VOLTAGE_NOMINAL_V 36.0f  /* docs/parameters.md, nominal voltage */
#define STUB_SHUNT_RESISTANCE_OHM 5.0e-4f /* docs/parameters.md, ASSUMED */
#define STUB_AMP_GAIN_V_PER_V 50.0f       /* docs/parameters.md, ASSUMED */
#define STUB_ADC_REFERENCE_V 3.3f         /* docs/parameters.md, ASSUMED */
#define STUB_ADC_FULL_SCALE_COUNTS 4095u  /* 12 bit, docs/parameters.md */
#define STUB_DEAD_TIME_S 5.0e-7f          /* 500 ns, docs/parameters.md, ASSUMED */
#define STUB_PWM_PERIOD_S 5.0e-5f         /* 1 / 20 kHz, s */
#define STUB_ENCODER_COUNTS_PER_REV 16384 /* 14 bit, docs/parameters.md */
#define STUB_POLE_PAIRS 7u                /* docs/parameters.md */
#define STUB_AMBIENT_C 25.0f              /* bench ambient, degC */
#define STUB_INDUCTANCE_H 2.56e-4f        /* phase inductance, parameters.md */
#define STUB_FLUX_LINKAGE_WB 4.39e-3f     /* flux linkage, parameters.md */
#define STUB_ROTOR_INERTIA_KG_M2 2.3e-6f  /* rotor inertia, parameters.md */

static hal_board_config_t s_board = {
    STUB_POLE_PAIRS,
    STUB_ENCODER_COUNTS_PER_REV,
    0,
    false,
    STUB_SHUNT_RESISTANCE_OHM,
    STUB_AMP_GAIN_V_PER_V,
    STUB_ADC_REFERENCE_V,
    (uint16_t)STUB_ADC_FULL_SCALE_COUNTS,
    STUB_BUS_VOLTAGE_NOMINAL_V,
    STUB_DEAD_TIME_S,
    STUB_PWM_PERIOD_S,
    STUB_INDUCTANCE_H,
    STUB_FLUX_LINKAGE_WB,
    /* The stub describes the bare motor: rotor inertia only, nothing hanging off
     * the shaft, so the speed loop gains scale by exactly one. */
    STUB_ROTOR_INERTIA_KG_M2,
    0.0f,
    0.0f, /* rigid transmission, no resonance limit in the stub */
    0.0f, /* and a noise free encoder, so the gains are the design's own */
};

static float s_duty[3] = {0.5f, 0.5f, 0.5f};
static bool s_pwm_enabled;
static uint32_t s_fault_bits;
static uint64_t s_time_us;
static uint32_t s_telemetry_drops;
static hal_tick_fn_t s_tick_fn;
static bool s_timing_running;
static uint32_t s_dead_time_ns;

hal_status_t hal_init(void) {
    s_pwm_enabled = false;
    s_fault_bits = 0u;
    s_time_us = 0u;
    s_telemetry_drops = 0u;
    s_dead_time_ns = 500u;
    return HAL_OK;
}

const hal_board_config_t *hal_board_config(void) {
    return &s_board;
}

hal_status_t hal_pwm_set_duty(float duty_a, float duty_b, float duty_c) {
    const float duty[3] = {duty_a, duty_b, duty_c};
    for (uint32_t i = 0u; i < 3u; ++i) {
        s_duty[i] = (duty[i] < 0.0f) ? 0.0f : ((duty[i] > 1.0f) ? 1.0f : duty[i]);
    }
    return HAL_OK;
}

void hal_pwm_enable(bool enable) {
    s_pwm_enabled = enable;
}

hal_status_t hal_pwm_set_dead_time_ns(uint32_t dead_time_ns) {
    s_dead_time_ns = dead_time_ns;
    return HAL_OK;
}

hal_status_t hal_current_read(hal_current_sample_t *out) {
    if (out == NULL) {
        return HAL_ERR_ARGUMENT;
    }
    out->ia_a = 0.0f;
    out->ib_a = 0.0f;
    out->ic_a = 0.0f;
    out->timestamp_us = (uint32_t)s_time_us;
    out->valid = true;
    return HAL_OK;
}

hal_status_t hal_encoder_read(hal_encoder_sample_t *out) {
    if (out == NULL) {
        return HAL_ERR_ARGUMENT;
    }
    out->position_counts = 0;
    out->timestamp_us = (uint32_t)s_time_us;
    out->index_seen = true;
    out->valid = true;
    return HAL_OK;
}

float hal_bus_voltage_read_v(void) {
    return STUB_BUS_VOLTAGE_NOMINAL_V;
}

float hal_temperature_read_c(void) {
    return STUB_AMBIENT_C;
}

uint32_t hal_fault_read(void) {
    return s_fault_bits;
}

void hal_fault_clear(void) {
    s_fault_bits = 0u;
}

void hal_fault_latch(uint32_t code) {
    s_fault_bits |= code;
}

void hal_watchdog_kick(void) {}

hal_status_t hal_watchdog_set_timeout_us(uint32_t timeout_us) {
    (void)timeout_us;
    return HAL_OK;
}

uint64_t hal_time_us(void) {
    s_time_us += 1u;
    return s_time_us;
}

uint32_t hal_cpu_cycles(void) {
    return 0u;
}

void hal_delay_us(uint32_t delay_us) {
    s_time_us += delay_us;
}

hal_status_t hal_set_control_tick(hal_tick_fn_t tick_fn) {
    s_tick_fn = tick_fn;
    return HAL_OK;
}

hal_status_t hal_start_control_timing(void) {
    s_timing_running = true;
    return HAL_OK;
}

void hal_stop_control_timing(void) {
    s_timing_running = false;
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
    s_telemetry_drops += 1u;
    return false;
}

uint32_t hal_telemetry_drop_count(void) {
    return s_telemetry_drops;
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

float hal_stub_last_duty(uint32_t phase_index) {
    if (phase_index >= 3u) {
        return 0.0f;
    }
    return s_duty[phase_index];
}

bool hal_stub_pwm_enabled(void) {
    return s_pwm_enabled;
}

uint32_t hal_stub_dead_time_ns(void) {
    return s_dead_time_ns;
}

bool hal_stub_timing_running(void) {
    return s_timing_running;
}

hal_tick_fn_t hal_stub_tick_fn(void) {
    return s_tick_fn;
}
