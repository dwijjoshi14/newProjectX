/* Field oriented control firmware. Day 1: the sensor to actuator path is wired
 * and gated to the zero vector. No controller is closed yet. */

#include "firmware.h"

#include <stddef.h>

#include "hal.h"

#define FIRMWARE_CURRENT_OFFSET_SAMPLES 256u /* boot calibration window, samples */
#define FIRMWARE_ZERO_VECTOR_DUTY 0.5f       /* all phases at 50 percent, duty */
#define FIRMWARE_TWO_PI_RAD 6.28318531f      /* 2 pi, rad */
#define FIRMWARE_ANGLE_LIMIT_RAD 1.0e7f      /* beyond this the encoder is wrong, rad */

static const hal_board_config_t *s_board = NULL;
static firmware_status_t s_status;
static float s_offset_ia_a;
static float s_offset_ib_a;
static float s_offset_ic_a;

/* Reduce an angle to [0, 2 pi). Bounded work, no library call. */
static float firmware_wrap_angle(float angle_rad) {
    if (!(angle_rad > -FIRMWARE_ANGLE_LIMIT_RAD && angle_rad < FIRMWARE_ANGLE_LIMIT_RAD)) {
        return 0.0f;
    }
    const int32_t turns = (int32_t)(angle_rad / FIRMWARE_TWO_PI_RAD);
    float wrapped_rad = angle_rad - (float)turns * FIRMWARE_TWO_PI_RAD;
    if (wrapped_rad < 0.0f) {
        wrapped_rad += FIRMWARE_TWO_PI_RAD;
    }
    if (wrapped_rad >= FIRMWARE_TWO_PI_RAD) {
        wrapped_rad -= FIRMWARE_TWO_PI_RAD;
    }
    return wrapped_rad;
}

/* Absolute value, A. Avoids a library call on the control path. */
static float firmware_abs(float value) {
    return (value < 0.0f) ? -value : value;
}

/* Average the phase current offsets with the bridge off. A, phase frame.
 * On the host the HAL returns one latched sample, so this is a no op until
 * the sensing rung models noise. */
static void firmware_calibrate_current_offsets(void) {
    hal_current_sample_t sample;
    float sum_ia_a = 0.0f;
    float sum_ib_a = 0.0f;
    float sum_ic_a = 0.0f;
    uint32_t count = 0u;

    for (uint32_t i = 0u; i < FIRMWARE_CURRENT_OFFSET_SAMPLES; ++i) {
        if (hal_current_read(&sample) == HAL_OK && sample.valid) {
            sum_ia_a += sample.ia_a;
            sum_ib_a += sample.ib_a;
            sum_ic_a += sample.ic_a;
            count += 1u;
        }
    }
    if (count > 0u) {
        const float inverse_count = 1.0f / (float)count;
        s_offset_ia_a = sum_ia_a * inverse_count;
        s_offset_ib_a = sum_ib_a * inverse_count;
        s_offset_ic_a = sum_ic_a * inverse_count;
    }
}

void firmware_init(void) {
    s_status.ticks = 0u;
    s_status.theta_e_rad = 0.0f;
    s_status.current_peak_a = 0.0f;
    s_status.fault_bits = 0u;
    s_offset_ia_a = 0.0f;
    s_offset_ib_a = 0.0f;
    s_offset_ic_a = 0.0f;

    if (hal_init() != HAL_OK) {
        hal_panic(1u, (uint32_t)__LINE__);
        return;
    }
    s_board = hal_board_config();
    if (s_board == NULL) {
        hal_panic(2u, (uint32_t)__LINE__);
        return;
    }

    hal_pwm_enable(false);
    firmware_calibrate_current_offsets();
    hal_pwm_enable(true);
    (void)hal_set_control_tick(firmware_tick);
}

void firmware_start(void) {
    (void)hal_start_control_timing();
}

void firmware_tick(void) {
    hal_current_sample_t current;
    hal_encoder_sample_t encoder;

    s_status.fault_bits = hal_fault_read();
    if (s_status.fault_bits != 0u) {
        hal_pwm_enable(false);
        hal_fault_latch(s_status.fault_bits);
        return;
    }

    float peak_a = 0.0f;
    if (hal_current_read(&current) == HAL_OK && current.valid) {
        const float ia_a = current.ia_a - s_offset_ia_a;
        const float ib_a = current.ib_a - s_offset_ib_a;
        const float ic_a = current.ic_a - s_offset_ic_a;
        peak_a = firmware_abs(ia_a);
        if (firmware_abs(ib_a) > peak_a) {
            peak_a = firmware_abs(ib_a);
        }
        if (firmware_abs(ic_a) > peak_a) {
            peak_a = firmware_abs(ic_a);
        }
    }
    s_status.current_peak_a = peak_a;

    if (hal_encoder_read(&encoder) == HAL_OK && encoder.valid && s_board != NULL) {
        int64_t counts = encoder.position_counts;
        if (s_board->encoder_inverted) {
            counts = -counts;
        }
        const float revolutions = (float)(counts - (int64_t)s_board->encoder_zero_offset_counts) /
                                  (float)s_board->encoder_counts_per_rev;
        const float theta_m_rad = revolutions * FIRMWARE_TWO_PI_RAD;
        s_status.theta_e_rad = firmware_wrap_angle(theta_m_rad * (float)s_board->pole_pairs);
    }

    /* Day 1 gate: the zero vector, no torque. The current loop arrives next. */
    (void)hal_pwm_set_duty(FIRMWARE_ZERO_VECTOR_DUTY, FIRMWARE_ZERO_VECTOR_DUTY,
                           FIRMWARE_ZERO_VECTOR_DUTY);
    hal_watchdog_kick();
    s_status.ticks += 1u;
}

const firmware_status_t *firmware_status(void) {
    return &s_status;
}
