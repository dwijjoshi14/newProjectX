/* Field oriented control firmware. Day 2: Clarke, Park, space vector
 * modulation, one current loop at 20 kHz and one speed loop at 2 kHz.
 *
 * Everything on the control path is float arithmetic in this repository, so
 * the portable target links no libm and the boundary gate still means
 * something. The gains below are design targets derived from cited motor
 * parameters. Whether they deliver what the design predicts is a measurement,
 * and it lives in docs/results.md.
 */

#include "firmware.h"

#include <stddef.h>

#include "foc.h"
#include "hal.h"
#include "pi.h"

#define FIRMWARE_CURRENT_OFFSET_SAMPLES 256u /* boot calibration window, samples */
#define FIRMWARE_ZERO_VECTOR_DUTY 0.5f       /* all phases at 50 percent, duty */
#define FIRMWARE_TWO_PI_RAD 6.28318548f      /* 2 pi, rad */
#define FIRMWARE_ANGLE_LIMIT_RAD 1.0e7f      /* beyond this the encoder is wrong, rad */
#define FIRMWARE_CONTROL_PERIOD_S 5.0e-5f    /* 20 kHz, s */
#define FIRMWARE_VELOCITY_PERIOD_S 5.0e-4f   /* 2 kHz, s */

/* Current loop. Kp sets the crossover at Kp / L: 1.61 V/A across 0.256 mH is
 * 1.0 kHz. Ki puts the PI zero on the winding pole R / L so the PI cancels it
 * exactly, which leaves one pole at the crossover. The 75 us pipeline delay
 * then costs about 27 degrees of phase, so the design margin is near 60.
 * Both numbers come from the datasheet row in docs/parameters.md, not from a
 * tuning session. */
#define FIRMWARE_CURRENT_KP_V_PER_A 1.61f
#define FIRMWARE_CURRENT_KI_V_PER_A_S 1830.0f

/* Speed loop, 80 Hz crossover against a pure inertia with 70 degrees of margin
 * before the sampling delay. Kp = J w / (Kt sqrt(1 + (wz / w)^2)) with
 * wz = w tan(20 deg). The plant has no damping term, so the margin comes
 * entirely from the PI zero.
 *
 * The crossover is not a preference. The first attempt used 12 Hz, computed
 * the same way, and a 50 mNm load step dropped the shaft from 200 rad/s to 39
 * before the loop reacted: the disturbance rate is T / J = 21700 rad/s^2 and
 * the rotor inertia is 2.3e-6 kg m^2, so a 50 mNm step moves this shaft faster
 * than any slow loop can answer. 100 Hz keeps the dip near what the delay
 * alone allows, 0.05 N m * 0.6 ms / J = 13 rad/s, while leaving the margin
 * about 50 degrees once the ten tick velocity estimate and the 2 kHz update
 * have taken their 30. */
#define FIRMWARE_SPEED_KP_A_PER_RAD_S 0.0313f
#define FIRMWARE_SPEED_KI_A_PER_RAD_S2 2.95f

/* Speed command filter, s. A step command on this loop arrives as a torque
 * step it cannot follow, and the answer is a 17 percent overshoot and a ring.
 * Filtering the command at 10 ms, a decade below the loop, spends nothing from
 * the voltage budget and costs 23 ms to reach 90 percent of a step, which is
 * still faster than a hand moves. */
#define FIRMWARE_SPEED_REFERENCE_FILTER_S 0.010f

#define FIRMWARE_CONTINUOUS_CURRENT_A 2.74f /* datasheet continuous current, A */
#define FIRMWARE_STALL_CURRENT_A 61.8f      /* datasheet stall current, A */
#define FIRMWARE_LINEAR_MODULATION 0.95f    /* fraction of bus / sqrt(3) used */
#define FIRMWARE_ONE_OVER_SQRT3 0.57735027f
#define FIRMWARE_BUS_MIN_V 6.0f     /* below this the gate holds the zero vector, V */
#define FIRMWARE_MAX_POLE_PAIRS 64u /* board config sanity limit, count */

static const hal_board_config_t *s_board = NULL;
static firmware_status_t s_status;
static float s_offset_ia_a;
static float s_offset_ib_a;
static float s_offset_ic_a;
static float s_rad_per_count;
static float s_current_reference_a;
static float s_speed_reference_rad_s;
static float s_iq_reference_a;
static float s_speed_command_rad_s;
static float s_voltage_limit_v;
static int64_t s_last_velocity_counts;
static uint32_t s_velocity_divider;
static pi_regulator_t s_pi_d;
static pi_regulator_t s_pi_q;
static pi_regulator_t s_pi_speed;

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

static float firmware_clamp(float value, float limit) {
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
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

/* Encoder counts in the rotor frame, after the direction and zero offset the
 * board config carries. */
static int64_t firmware_counts(const hal_encoder_sample_t *encoder) {
    int64_t counts = encoder->position_counts;
    if (s_board->encoder_inverted) {
        counts = -counts;
    }
    return counts - (int64_t)s_board->encoder_zero_offset_counts;
}

void firmware_init(void) {
    s_status.ticks = 0u;
    s_status.mode = FIRMWARE_MODE_OFF;
    s_status.theta_e_rad = 0.0f;
    s_status.current_peak_a = 0.0f;
    s_status.omega_rad_s = 0.0f;
    s_status.iq_ref_a = 0.0f;
    s_status.id_a = 0.0f;
    s_status.iq_a = 0.0f;
    s_status.vd_v = 0.0f;
    s_status.vq_v = 0.0f;
    s_status.bus_v = 0.0f;
    s_status.fault_bits = 0u;
    s_offset_ia_a = 0.0f;
    s_offset_ib_a = 0.0f;
    s_offset_ic_a = 0.0f;
    s_rad_per_count = 0.0f;
    s_current_reference_a = 0.0f;
    s_speed_reference_rad_s = 0.0f;
    s_iq_reference_a = 0.0f;
    s_speed_command_rad_s = 0.0f;
    s_voltage_limit_v = 0.0f;
    s_last_velocity_counts = 0;
    s_velocity_divider = 0u;

    pi_init(&s_pi_d, FIRMWARE_CURRENT_KP_V_PER_A, FIRMWARE_CURRENT_KI_V_PER_A_S,
            FIRMWARE_CONTROL_PERIOD_S, 0.0f);
    pi_init(&s_pi_q, FIRMWARE_CURRENT_KP_V_PER_A, FIRMWARE_CURRENT_KI_V_PER_A_S,
            FIRMWARE_CONTROL_PERIOD_S, 0.0f);
    pi_init(&s_pi_speed, FIRMWARE_SPEED_KP_A_PER_RAD_S, FIRMWARE_SPEED_KI_A_PER_RAD_S2,
            FIRMWARE_VELOCITY_PERIOD_S, FIRMWARE_CONTINUOUS_CURRENT_A);

    if (hal_init() != HAL_OK) {
        hal_panic(1u, (uint32_t)__LINE__);
        return;
    }
    s_board = hal_board_config();
    if (s_board == NULL || s_board->encoder_counts_per_rev <= 0 || s_board->pole_pairs == 0u ||
        s_board->pole_pairs > FIRMWARE_MAX_POLE_PAIRS) {
        hal_panic(2u, (uint32_t)__LINE__);
        return;
    }
    s_rad_per_count = FIRMWARE_TWO_PI_RAD / (float)s_board->encoder_counts_per_rev;

    hal_pwm_enable(false);
    firmware_calibrate_current_offsets();
    hal_pwm_enable(true);

    /* Seed the velocity estimate, so the first update is not measured from
     * whatever angle the rotor happened to be parked at. */
    hal_encoder_sample_t encoder;
    if (hal_encoder_read(&encoder) == HAL_OK && encoder.valid) {
        s_last_velocity_counts = firmware_counts(&encoder);
    }

    (void)hal_set_control_tick(firmware_tick);
}

void firmware_start(void) {
    (void)hal_start_control_timing();
}

void firmware_set_mode(firmware_mode_t mode) {
    s_status.mode = mode;
    s_iq_reference_a = 0.0f;
    /* Start the slaved command where the shaft is, so the first tick after a
     * mode change is not a step. */
    s_speed_command_rad_s = s_status.omega_rad_s;
    pi_reset(&s_pi_d);
    pi_reset(&s_pi_q);
    pi_reset(&s_pi_speed);
}

void firmware_set_current_reference_a(float iq_a) {
    /* Clamped at stall current, not at continuous. A joint asked for a peak
     * torque has to be able to answer; the thermal network that would make the
     * continuous number a real limit arrives on day 5. */
    s_current_reference_a = firmware_clamp(iq_a, FIRMWARE_STALL_CURRENT_A);
}

void firmware_set_speed_reference_rad_s(float omega_rad_s) {
    s_speed_reference_rad_s = omega_rad_s;
}

void firmware_tick(void) {
    hal_current_sample_t current;
    hal_encoder_sample_t encoder;
    float ia_a = 0.0f;
    float ib_a = 0.0f;
    float ic_a = 0.0f;
    bool speed_update = false;

    s_status.fault_bits = hal_fault_read();
    if (s_status.fault_bits != 0u) {
        hal_pwm_enable(false);
        hal_fault_latch(s_status.fault_bits);
        return;
    }

    if (hal_current_read(&current) == HAL_OK && current.valid) {
        ia_a = current.ia_a - s_offset_ia_a;
        ib_a = current.ib_a - s_offset_ib_a;
        ic_a = current.ic_a - s_offset_ic_a;
    }
    float peak_a = firmware_abs(ia_a);
    if (firmware_abs(ib_a) > peak_a) {
        peak_a = firmware_abs(ib_a);
    }
    if (firmware_abs(ic_a) > peak_a) {
        peak_a = firmware_abs(ic_a);
    }
    s_status.current_peak_a = peak_a;

    if (hal_encoder_read(&encoder) == HAL_OK && encoder.valid) {
        const int64_t counts = firmware_counts(&encoder);
        const float theta_m_rad = (float)counts * s_rad_per_count;
        s_status.theta_e_rad = firmware_wrap_angle(theta_m_rad * (float)s_board->pole_pairs);

        /* The velocity loop runs every tenth tick. The estimate divides by the
         * nominal window rather than by the measured timestamp, so it carries
         * no clock jitter. Encoder quantisation at 14 bits is the dominant
         * error either way, and a board's timer period is as fixed as this. */
        s_velocity_divider += 1u;
        if (s_velocity_divider >= HAL_VELOCITY_DIVIDER) {
            const float delta_rad = (float)(counts - s_last_velocity_counts) * s_rad_per_count;
            s_status.omega_rad_s = delta_rad / FIRMWARE_VELOCITY_PERIOD_S;
            s_last_velocity_counts = counts;
            s_velocity_divider = 0u;
            speed_update = true;
        }
    }

    const float bus_v = hal_bus_voltage_read_v();
    s_status.bus_v = bus_v;
    s_voltage_limit_v = (bus_v > FIRMWARE_BUS_MIN_V)
                            ? FIRMWARE_LINEAR_MODULATION * FIRMWARE_ONE_OVER_SQRT3 * bus_v
                            : 0.0f;

    const foc_rotor_t rotor = foc_rotor(s_status.theta_e_rad);
    const foc_alphabeta_t current_ab = foc_clarke((foc_phase_t){ia_a, ib_a, ic_a});
    const foc_dq_t current_dq = foc_park(current_ab, rotor);
    s_status.id_a = current_dq.d;
    s_status.iq_a = current_dq.q;

    if (s_status.mode == FIRMWARE_MODE_OFF) {
        /* Zero vector: all three phases at 50 percent, so no differential
         * voltage reaches the winding and the bridge is in a known state. */
        s_iq_reference_a = 0.0f;
        s_status.iq_ref_a = 0.0f;
        s_status.vd_v = 0.0f;
        s_status.vq_v = 0.0f;
        pi_reset(&s_pi_d);
        pi_reset(&s_pi_q);
        pi_reset(&s_pi_speed);
        (void)hal_pwm_set_duty(FIRMWARE_ZERO_VECTOR_DUTY, FIRMWARE_ZERO_VECTOR_DUTY,
                               FIRMWARE_ZERO_VECTOR_DUTY);
        hal_watchdog_kick();
        s_status.ticks += 1u;
        return;
    }

    if (s_status.mode == FIRMWARE_MODE_SPEED) {
        if (speed_update) {
            /* First order command filter, one update per speed tick. */
            const float filter_alpha =
                FIRMWARE_VELOCITY_PERIOD_S / FIRMWARE_SPEED_REFERENCE_FILTER_S;
            s_speed_command_rad_s +=
                (s_speed_reference_rad_s - s_speed_command_rad_s) * filter_alpha;
            s_iq_reference_a = pi_step(&s_pi_speed, s_speed_command_rad_s - s_status.omega_rad_s);
        }
    } else {
        s_iq_reference_a = s_current_reference_a;
    }
    s_status.iq_ref_a = s_iq_reference_a;

    pi_set_limit(&s_pi_d, s_voltage_limit_v);
    pi_set_limit(&s_pi_q, s_voltage_limit_v);
    const foc_dq_t voltage_command =
        foc_limit_magnitude((foc_dq_t){pi_step(&s_pi_d, 0.0f - current_dq.d),
                                       pi_step(&s_pi_q, s_iq_reference_a - current_dq.q)},
                            s_voltage_limit_v);
    s_status.vd_v = voltage_command.d;
    s_status.vq_v = voltage_command.q;

    const foc_alphabeta_t voltage_ab = foc_inverse_park(voltage_command, rotor);
    const foc_phase_t duty = foc_svpwm(voltage_ab, bus_v);
    (void)hal_pwm_set_duty(duty.a, duty.b, duty.c);
    hal_watchdog_kick();
    s_status.ticks += 1u;
}

const firmware_status_t *firmware_status(void) {
    return &s_status;
}
