/* Field oriented control firmware. Clarke, Park, space vector modulation, a
 * current loop at 20 kHz and a speed loop at 2 kHz, plus the two voltage terms
 * a drive needs once the inverter is real: d/q decoupling with back EMF
 * feedforward, and gate dead time compensation.
 *
 * Both of those need the motor parameters and the gate dead time, which arrive
 * through hal_board_config. Nothing in this file is specific to a simulator,
 * a board or a motor: change the config and the same source runs elsewhere.
 *
 * Everything on the control path is float arithmetic, so the portable target
 * links no libm and the boundary gate still means something. The gains below
 * are design targets derived from cited motor parameters. Whether they deliver
 * what the design predicts is a measurement, and it lives in docs/results.md.
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
#define FIRMWARE_SPEED_CROSSOVER_RAD_S 628.31853f /* 100 Hz, the design above */

/* Both of those are for the motor's own rotor inertia, 2.3e-6 kg m^2, closing at
 * 100 Hz. They are scaled at init by J_total / J_rotor and by the crossover the
 * joint can take, because Kp = J wc / Kt and Ki = Kp wz are both proportional to
 * J and to wc. Day 4 is why the second factor exists: a coupling that rings at
 * 100 Hz cannot have a speed loop crossing at 100 Hz, and this firmware used to
 * try, on a joint whose 0.05 N m s/rad of assumed coupling damping made the
 * transmission look rigid enough that the mistake did not show. */

/* Speed command filter, s. A step command on this loop arrives as a torque
 * step it cannot follow, and the answer is a 17 percent overshoot and a ring.
 * Filtering the command at 10 ms, a decade below the loop, spends nothing from
 * the voltage budget and costs 23 ms to reach 90 percent of a step, which is
 * still faster than a hand moves. */
#define FIRMWARE_SPEED_REFERENCE_FILTER_S 0.010f

/* Position loop, 1 kHz, proportional on position into the speed loop's
 * reference. 157 per second is a 25 Hz crossover: the speed loop underneath
 * holds to about 100 Hz, so at 25 Hz its own lag is small and the margin is
 * near 80 degrees before the encoder. A position loop with no integral is
 * enough because the speed loop already has one, which is what makes a cascade
 * worth the trouble.
 *
 * The speed command is not passed through the 10 ms reference filter the speed
 * mode uses: that filter is there to stop a step command exciting the speed
 * loop, and a position loop's output is already smooth. Filtering it again
 * would put a 16 Hz pole inside a 25 Hz loop. */
#define FIRMWARE_POSITION_KP_PER_S 157.0f
#define FIRMWARE_POSITION_MAX_SPEED_RAD_S 400.0f /* clamp on the cascade output, rad/s */
#define FIRMWARE_POSITION_LIMIT_RAD 1.0e6f       /* beyond this the reference is wrong, rad */

#define FIRMWARE_CONTINUOUS_CURRENT_A 2.74f /* datasheet continuous current, A */
#define FIRMWARE_STALL_CURRENT_A 61.8f      /* datasheet stall current, A */
/* Below this phase current the dead time compensation fades out linearly.
 * The sign of the current is not single valued inside one switching period
 * once the ripple crosses zero, so a hard sign there would add distortion in
 * the name of removing it. Half the predicted 1.76 A ripple is the point where
 * the sign starts to be trustworthy. */
#define FIRMWARE_DEAD_TIME_LINEAR_A 0.5f

/* Thermal. The winding's insulation limit is 155 degC and the drive opens its
 * gates 25 K below it, because a sensor that reads the average winding is not
 * the hottest turn and a control that only acts at the limit has no margin to
 * act in. Between 100 and 130 degC the current envelope falls linearly from
 * stall to nothing, which is not a derating curve: it is the envelope pulled
 * down so that a drive already too hot cannot be asked for a peak. The
 * derating curve for continuous operation is a property of the machine and it
 * is computed from the network in docs/results.md, not from this constant. */
#define FIRMWARE_OVERTEMP_LIMIT_C 130.0f       /* 155 degC insulation minus 25 K */
#define FIRMWARE_THERMAL_DERATE_START_C 100.0f /* where the envelope starts to fall */
#define FIRMWARE_THERMAL_DIVIDER 100u          /* temperature read at 200 Hz, ticks */

/* Passivity controller. The damping is the excess energy divided by the speed
 * squared and the tick, which is the amount that dissipates exactly the excess
 * in one tick. The cap matters: a speed near zero makes that division
 * singular, and an uncapped beta would ask for a torque the drive cannot
 * produce and call the result passive. */
#define FIRMWARE_PASSIVITY_MAX_DAMPING 50.0f /* N m s/rad */
#define FIRMWARE_PASSIVITY_MIN_SPEED_SQUARED 1.0e-6f
#define FIRMWARE_LINEAR_MODULATION 0.95f /* fraction of bus / sqrt(3) used */
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
static int64_t s_position_reference_counts;
static uint32_t s_velocity_divider;
static uint32_t s_position_divider;
static float s_position_error_rad;
static float s_position_kp_per_s;
static float s_stiffness_nm_per_rad;
static float s_damping_nm_s_per_rad;
static float s_torque_constant_nm_per_a;
static bool s_dead_time_compensation;
static float s_temperature_c;
static float s_thermal_limit_a;
static uint32_t s_thermal_divider;
static bool s_passivity_enabled;
static float s_passivity_energy_j;
static float s_passivity_excess_j;
static float s_passivity_damping_nm_s_per_rad;
/* The far end's accumulated port energy, J, as the network last reported it.
 * Zero is a one port render, which is what every run before day 6 was. */
static float s_link_energy_j;
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

/* The current envelope at a winding temperature, A. At a bench temperature
 * this is the stall current and it changes nothing, which is why day 1 to day 4
 * results do not move when this arrives. */
static float firmware_thermal_limit(float temperature_c) {
    if (!(temperature_c > FIRMWARE_THERMAL_DERATE_START_C)) {
        return FIRMWARE_STALL_CURRENT_A;
    }
    if (!(temperature_c < FIRMWARE_OVERTEMP_LIMIT_C)) {
        return 0.0f;
    }
    return FIRMWARE_STALL_CURRENT_A * (FIRMWARE_OVERTEMP_LIMIT_C - temperature_c) /
           (FIRMWARE_OVERTEMP_LIMIT_C - FIRMWARE_THERMAL_DERATE_START_C);
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

/* Radians to counts, rounded away from zero. C truncates toward zero, so the
 * half count is added explicitly; writing it as a cast of (x + 0.5f) anyway
 * would round a negative angle the wrong way, which is the kind of one count
 * offset that turns into a position loop with a steady state error. */
static int64_t firmware_rad_to_counts(float theta_rad) {
    if (!(s_rad_per_count > 0.0f)) {
        return 0;
    }
    const float counts = firmware_clamp(theta_rad, FIRMWARE_POSITION_LIMIT_RAD) / s_rad_per_count;
    if (counts >= 0.0f) {
        return (int64_t)(counts + 0.5f);
    }
    return (int64_t)(counts - 0.5f);
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
    s_status.vd_ff_v = 0.0f;
    s_status.vq_ff_v = 0.0f;
    s_status.bus_v = 0.0f;
    s_status.dead_time_compensation = true;
    s_status.fault_bits = 0u;
    s_status.position_rad = 0.0f;
    s_status.position_ref_rad = 0.0f;
    s_status.position_error_rad = 0.0f;
    s_status.torque_ref_nm = 0.0f;
    s_status.stiffness_nm_per_rad = 0.0f;
    s_status.damping_nm_s_per_rad = 0.0f;
    s_status.temperature_c = 0.0f;
    s_status.thermal_limit_a = FIRMWARE_STALL_CURRENT_A;
    s_status.passivity_energy_j = 0.0f;
    s_status.passivity_excess_j = 0.0f;
    s_status.passivity_damping_nm_s_per_rad = 0.0f;
    s_status.passivity_peer_energy_j = 0.0f;
    s_temperature_c = 0.0f;
    s_thermal_limit_a = FIRMWARE_STALL_CURRENT_A;
    s_thermal_divider = 0u;
    s_passivity_enabled = false;
    s_passivity_energy_j = 0.0f;
    s_passivity_excess_j = 0.0f;
    s_passivity_damping_nm_s_per_rad = 0.0f;
    s_link_energy_j = 0.0f;
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
    s_position_reference_counts = 0;
    s_velocity_divider = 0u;
    s_position_divider = 0u;
    s_position_error_rad = 0.0f;
    s_position_kp_per_s = FIRMWARE_POSITION_KP_PER_S;
    s_stiffness_nm_per_rad = 0.0f;
    s_damping_nm_s_per_rad = 0.0f;
    s_torque_constant_nm_per_a = 0.0f;
    s_dead_time_compensation = true;

    pi_init(&s_pi_d, FIRMWARE_CURRENT_KP_V_PER_A, FIRMWARE_CURRENT_KI_V_PER_A_S,
            FIRMWARE_CONTROL_PERIOD_S, 0.0f);
    pi_init(&s_pi_q, FIRMWARE_CURRENT_KP_V_PER_A, FIRMWARE_CURRENT_KI_V_PER_A_S,
            FIRMWARE_CONTROL_PERIOD_S, 0.0f);

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

    /* Torque per amp, from the same two motor constants the feedforward uses.
     * The impedance mode needs it and the firmware has no other place to get
     * it: Kt is not a separate board config field, it is 1.5 p lambda. */
    s_torque_constant_nm_per_a = 1.5f * (float)s_board->pole_pairs * s_board->motor_flux_linkage_wb;
    if (!(s_torque_constant_nm_per_a > 0.0f)) {
        hal_panic(3u, (uint32_t)__LINE__);
        return;
    }

    /* One temperature read at boot, so the envelope is right before the first
     * 5 ms has passed. A board whose thermistor is not wired reads ambient and
     * gets the stall current, which is the honest answer for a sensor that says
     * nothing. */
    s_temperature_c = hal_temperature_read_c();
    s_thermal_limit_a = firmware_thermal_limit(s_temperature_c);
    s_status.temperature_c = s_temperature_c;
    s_status.thermal_limit_a = s_thermal_limit_a;

    /* Speed loop gains from the inertia the joint actually presents. A board
     * that has not been told its load inertia gets the rotor alone, which is
     * what day 2 measured and what day 1 to day 3 published. */
    float inertia_ratio = 1.0f;
    const float inertia_kg_m2 =
        s_board->motor_inertia_kg_m2 + s_board->reflected_load_inertia_kg_m2;
    if (s_board->motor_inertia_kg_m2 > 0.0f) {
        inertia_ratio = inertia_kg_m2 / s_board->motor_inertia_kg_m2;
    }

    /* How fast this joint is allowed to close, which is not a preference. The
     * hand on the coupling resonates at sqrt(k / J_load), the anti resonance,
     * and that is the first thing in the plant that turns the loop's phase over.
     * A velocity loop has to stay well below it; a quarter of it is the usual
     * place to stop. With a rigid transmission the board reports no stiffness
     * and the design 100 Hz stands, which is what day 1 to day 3 measured. With
     * the day 4 handle on a steel shaft that anti resonance is 205 Hz, so the
     * drive commissions itself to 51. */
    float allowed_rad_s = FIRMWARE_SPEED_CROSSOVER_RAD_S;
    if (s_board->coupling_stiffness_nm_per_rad > 0.0f &&
        s_board->reflected_load_inertia_kg_m2 > 0.0f) {
        const float anti_resonance_rad_s = foc_sqrt(s_board->coupling_stiffness_nm_per_rad /
                                                    s_board->reflected_load_inertia_kg_m2);
        if (0.25f * anti_resonance_rad_s < allowed_rad_s) {
            allowed_rad_s = 0.25f * anti_resonance_rad_s;
        }
    }

    /* And how fast the sensor allows, which on a loaded joint is the tighter of
     * the two. The velocity estimate is a difference of two encoder samples one
     * velocity period apart, so its noise is sqrt(2) sigma / T. A speed loop
     * multiplies that by its own Kp, and Kp is J wc / Kt, so the current the
     * noise alone commands is J wc sigma_omega / Kt. Holding that to a tenth of
     * continuous current gives the fastest loop this encoder can support. It is
     * not a tuning preference: below this the noise current is smaller than the
     * friction this joint has to overcome, and above it the loop drives the
     * bridge with its own sensor noise. */
    if (s_board->encoder_noise_rad > 0.0f && inertia_kg_m2 > 0.0f) {
        const float velocity_noise_rad_s =
            1.41421356f * s_board->encoder_noise_rad / FIRMWARE_VELOCITY_PERIOD_S;
        const float sensor_limited_rad_s = 0.1f * FIRMWARE_CONTINUOUS_CURRENT_A *
                                           s_torque_constant_nm_per_a /
                                           (inertia_kg_m2 * velocity_noise_rad_s);
        if (sensor_limited_rad_s < allowed_rad_s) {
            allowed_rad_s = sensor_limited_rad_s;
        }
    }
    const float crossover_ratio = allowed_rad_s / FIRMWARE_SPEED_CROSSOVER_RAD_S;

    pi_init(&s_pi_speed, FIRMWARE_SPEED_KP_A_PER_RAD_S * inertia_ratio * crossover_ratio,
            FIRMWARE_SPEED_KI_A_PER_RAD_S2 * inertia_ratio * crossover_ratio,
            FIRMWARE_VELOCITY_PERIOD_S, FIRMWARE_CONTINUOUS_CURRENT_A);

    /* The position loop sits a quarter of the way up the speed loop, which is
     * the ratio the 157 per second was chosen with. If the joint's mechanics or
     * its encoder take the speed loop down, the position loop comes down with it
     * or the cascade stops being a cascade. */
    s_position_kp_per_s = FIRMWARE_POSITION_KP_PER_S * crossover_ratio;
    s_status.position_kp_per_s = s_position_kp_per_s;
    s_status.speed_crossover_rad_s = allowed_rad_s;

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
    /* A passivity observer carries no energy across a mode change: the render it
     * was watching no longer exists, and an accumulated deficit from the last
     * one would ask the new one for damping it has not earned. */
    s_passivity_energy_j = 0.0f;
    s_passivity_excess_j = 0.0f;
    s_passivity_damping_nm_s_per_rad = 0.0f;
    /* The far end's number belongs to the render that is gone as well. A drive
     * that kept it would damp its next render for a debt another controller ran
     * up, and a two port pair that changed mode would spend a second shaking off
     * a coefficient that no longer describes anything. */
    s_link_energy_j = 0.0f;
    s_status.passivity_peer_energy_j = 0.0f;
    /* Start the slaved command where the shaft is, so the first tick after a
     * mode change is not a step. */
    s_speed_command_rad_s = s_status.omega_rad_s;
    s_position_divider = 0u;
    pi_reset(&s_pi_d);
    pi_reset(&s_pi_q);
    pi_reset(&s_pi_speed);

    /* Same idea for position: entering the mode holds the joint where it is
     * instead of commanding a move to zero. A joint that jumps on a mode change
     * is a joint somebody drops. */
    hal_encoder_sample_t encoder;
    if (hal_encoder_read(&encoder) == HAL_OK && encoder.valid) {
        s_position_reference_counts = firmware_counts(&encoder);
        s_position_error_rad = 0.0f;
        s_status.position_error_rad = 0.0f;
        s_status.position_ref_rad = (float)s_position_reference_counts * s_rad_per_count;
    }
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

void firmware_set_dead_time_compensation(bool enable) {
    s_dead_time_compensation = enable;
    s_status.dead_time_compensation = enable;
}

void firmware_set_position_reference_rad(float theta_rad) {
    s_position_reference_counts = firmware_rad_to_counts(theta_rad);
    s_status.position_ref_rad = (float)s_position_reference_counts * s_rad_per_count;
}

void firmware_set_passivity_control(bool enable) {
    s_passivity_enabled = enable;
    s_passivity_energy_j = 0.0f;
    s_passivity_excess_j = 0.0f;
}

void firmware_set_link_energy_j(float energy_j) {
    s_link_energy_j = energy_j;
    s_status.passivity_peer_energy_j = energy_j;
}

void firmware_set_impedance(float stiffness_nm_per_rad, float damping_nm_s_per_rad) {
    /* A negative spring would push the joint away from the handle, which is not
     * a thing a haptic interface should do by accident. NaN fails both
     * comparisons and lands on zero. */
    s_stiffness_nm_per_rad = (stiffness_nm_per_rad >= 0.0f) ? stiffness_nm_per_rad : 0.0f;
    s_damping_nm_s_per_rad = (damping_nm_s_per_rad >= 0.0f) ? damping_nm_s_per_rad : 0.0f;
    s_status.stiffness_nm_per_rad = s_stiffness_nm_per_rad;
    s_status.damping_nm_s_per_rad = s_damping_nm_s_per_rad;
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

    /* Temperature at 200 Hz, which is a hundred times faster than the winding's
     * 25 s time constant and 1/25 of the cost of reading it every tick. At the
     * limit the gates open here rather than in the fault block above, because
     * the fault bit is latched by this firmware and read back on the next tick;
     * waiting for that tick would leave a bridge on for 50 us longer than the
     * limit allows, which is harmless, and would also report a running drive for
     * one tick after it stopped, which is not. */
    s_thermal_divider += 1u;
    if (s_thermal_divider >= FIRMWARE_THERMAL_DIVIDER) {
        s_thermal_divider = 0u;
        s_temperature_c = hal_temperature_read_c();
        s_thermal_limit_a = firmware_thermal_limit(s_temperature_c);
        s_status.temperature_c = s_temperature_c;
        s_status.thermal_limit_a = s_thermal_limit_a;
        if (!(s_temperature_c < FIRMWARE_OVERTEMP_LIMIT_C)) {
            hal_fault_latch(HAL_FAULT_OVERTEMP);
            s_status.fault_bits = hal_fault_read();
            hal_pwm_enable(false);
            return;
        }
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

    bool counts_valid = false;
    int64_t counts_now = 0;
    if (hal_encoder_read(&encoder) == HAL_OK && encoder.valid) {
        const int64_t counts = firmware_counts(&encoder);
        const float theta_m_rad = (float)counts * s_rad_per_count;
        s_status.theta_e_rad = firmware_wrap_angle(theta_m_rad * (float)s_board->pole_pairs);
        counts_now = counts;
        counts_valid = true;
        s_status.position_rad = theta_m_rad;

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

    /* The position error is computed in counts and only then converted, so a
     * joint that has run a thousand turns does not lose the last counts to the
     * mantissa of a float angle. The reference is absolute counts too, which is
     * what a multiturn counter on a 14 bit encoder hands you. */
    if (counts_valid) {
        const int64_t error_counts = s_position_reference_counts - counts_now;
        s_position_error_rad = (float)error_counts * s_rad_per_count;
        s_status.position_error_rad = s_position_error_rad;
    }

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
        s_status.vd_ff_v = 0.0f;
        s_status.vq_ff_v = 0.0f;
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
    } else if (s_status.mode == FIRMWARE_MODE_POSITION) {
        /* Position loop at 1 kHz writes the speed loop's reference; the speed PI
         * keeps running at 2 kHz on whatever that reference last was, so the
         * inner loop is not slowed down to the outer one's rate. On a tick where
         * both run, the position update goes first. */
        s_position_divider += 1u;
        if (s_position_divider >= HAL_POSITION_DIVIDER) {
            s_position_divider = 0u;
            s_speed_command_rad_s = firmware_clamp(s_position_kp_per_s * s_position_error_rad,
                                                   FIRMWARE_POSITION_MAX_SPEED_RAD_S);
            s_status.speed_command_rad_s = s_speed_command_rad_s;
        }
        if (speed_update) {
            s_iq_reference_a = pi_step(&s_pi_speed, s_speed_command_rad_s - s_status.omega_rad_s);
        }
    } else if (s_status.mode == FIRMWARE_MODE_IMPEDANCE) {
        /* A spring and a damper, at the full 20 kHz because that is what a
         * haptic joint needs: the handle's impedance is the loop, and a slow one
         * feels like a soft one. The damping term is only as good as the speed
         * estimate multiplied into it, which is a ten tick difference behind the
         * truth, so a stiff damper here lags. The observer that would fix that
         * does not exist yet. */
        const float velocity_rad_s = s_status.omega_rad_s;
        float torque_nm =
            s_stiffness_nm_per_rad * s_position_error_rad - s_damping_nm_s_per_rad * velocity_rad_s;
        {
            /* Time domain passivity observer. It runs whenever the impedance
             * mode does, because it is an observation and costs one multiply;
             * only the controller that acts on it is optional. The energy that
             * crosses the port
             * is torque times speed integrated, and for a spring under a fixed
             * reference it is exactly minus the stored energy: the render takes
             * energy in as it is pushed and gives it back as it returns. Adding
             * the stored energy back therefore leaves the energy the render has
             * actually dissipated, which cannot be negative in a passive one.
             *
             * It goes negative when the reference moves without work being done
             * against it, and that is the whole failure mode of a bilateral pair
             * over a link: the far handle's angle arrives 2 ms late, the spring
             * is stretched by a position it did not see, and the drive delivers
             * energy it never stored. */
            s_passivity_energy_j += torque_nm * velocity_rad_s * FIRMWARE_CONTROL_PERIOD_S;
            const float stored_j =
                0.5f * s_stiffness_nm_per_rad * s_position_error_rad * s_position_error_rad;
            /* Two ports. This drive can only measure the energy that crosses
             * its own port, and the failure being watched for is a pair that
             * generates: the master pushes energy out, the far render is late,
             * and the far port is where it comes back. Adding the far end's
             * number, as the link delivered it, is the whole of the difference.
             * With no link the term is zero and this is the day 5 observer. */
            float excess_j = -(s_passivity_energy_j + s_link_energy_j + stored_j);
            float damping_nm_s_per_rad = 0.0f;
            if (s_passivity_enabled && excess_j < 0.0f) {
                const float speed_squared = velocity_rad_s * velocity_rad_s;
                if (speed_squared > FIRMWARE_PASSIVITY_MIN_SPEED_SQUARED) {
                    damping_nm_s_per_rad = -excess_j / (speed_squared * FIRMWARE_CONTROL_PERIOD_S);
                    if (damping_nm_s_per_rad > FIRMWARE_PASSIVITY_MAX_DAMPING) {
                        damping_nm_s_per_rad = FIRMWARE_PASSIVITY_MAX_DAMPING;
                    }
                    /* The damping the observer asks for is a torque, and the
                     * drive can only produce stall. Asking for 290 N m from a
                     * 2.85 N m motor and then subtracting the energy as if it had
                     * been delivered is how a controller reports itself passive
                     * while the joint carries on oscillating; this project
                     * measured exactly that before the clamp existed. The
                     * correction is limited to the torque the drive can make and
                     * the observer is charged for what was actually applied, so a
                     * render it cannot save stays negative and says so. */
                    float correction_nm = -damping_nm_s_per_rad * velocity_rad_s;
                    const float torque_limit_nm =
                        FIRMWARE_STALL_CURRENT_A * s_torque_constant_nm_per_a;
                    if (correction_nm > torque_limit_nm) {
                        correction_nm = torque_limit_nm;
                    }
                    if (correction_nm < -torque_limit_nm) {
                        correction_nm = -torque_limit_nm;
                    }
                    torque_nm += correction_nm;
                    s_passivity_energy_j +=
                        correction_nm * velocity_rad_s * FIRMWARE_CONTROL_PERIOD_S;
                    damping_nm_s_per_rad = -correction_nm / velocity_rad_s;
                    excess_j = -(s_passivity_energy_j + s_link_energy_j + stored_j);
                }
            }
            s_passivity_damping_nm_s_per_rad = damping_nm_s_per_rad;
            s_passivity_excess_j = excess_j;
            s_status.passivity_energy_j = s_passivity_energy_j;
            s_status.passivity_excess_j = s_passivity_excess_j;
            s_status.passivity_damping_nm_s_per_rad = s_passivity_damping_nm_s_per_rad;
        }
        s_status.torque_ref_nm = torque_nm;
        s_iq_reference_a =
            firmware_clamp(torque_nm / s_torque_constant_nm_per_a, FIRMWARE_STALL_CURRENT_A);
    } else {
        s_iq_reference_a = s_current_reference_a;
    }
    /* The thermal envelope on top of whatever the mode asked for. At a bench
     * temperature the limit is the stall current, so this comparison never
     * fires and every published run is unchanged. It exists for the run where
     * the winding is at 120 degC and the impedance mode is still asking for a
     * peak. */
    if (firmware_abs(s_iq_reference_a) > s_thermal_limit_a) {
        s_iq_reference_a = (s_iq_reference_a > 0.0f) ? s_thermal_limit_a : -s_thermal_limit_a;
    }
    s_status.iq_ref_a = s_iq_reference_a;

    /* Decoupling and back EMF feedforward. The voltage a rotating machine
     * needs is not the voltage its current error asks for: the d axis has to
     * supply -omega_e L iq, and the q axis omega_e L id plus the back EMF. The
     * PI can reject both, and did until day 3, at the price of a d axis
     * excursion during every speed transient. Feeding them forward takes the
     * job off the integrator and leaves it for disturbances, which is what it
     * is for. omega_e comes from the 2 kHz velocity estimate, the fastest speed
     * this firmware has. */
    const float omega_e_rad_s = (float)s_board->pole_pairs * s_status.omega_rad_s;
    const float vd_ff_v = -omega_e_rad_s * s_board->motor_inductance_h * current_dq.q;
    const float vq_ff_v = omega_e_rad_s * s_board->motor_inductance_h * current_dq.d +
                          omega_e_rad_s * s_board->motor_flux_linkage_wb;
    s_status.vd_ff_v = vd_ff_v;
    s_status.vq_ff_v = vq_ff_v;

    /* The PI now only has to cover what is left of the voltage budget. Limiting
     * it to the full budget instead would let the integrator wind up around a
     * voltage it never gets to apply, because the feedforward already spends
     * part of it. */
    const float remaining_d_v = (s_voltage_limit_v > firmware_abs(vd_ff_v))
                                    ? s_voltage_limit_v - firmware_abs(vd_ff_v)
                                    : 0.0f;
    const float remaining_q_v = (s_voltage_limit_v > firmware_abs(vq_ff_v))
                                    ? s_voltage_limit_v - firmware_abs(vq_ff_v)
                                    : 0.0f;
    pi_set_limit(&s_pi_d, remaining_d_v);
    pi_set_limit(&s_pi_q, remaining_q_v);
    const foc_dq_t voltage_command =
        foc_limit_magnitude((foc_dq_t){pi_step(&s_pi_d, 0.0f - current_dq.d) + vd_ff_v,
                                       pi_step(&s_pi_q, s_iq_reference_a - current_dq.q) + vq_ff_v},
                            s_voltage_limit_v);
    s_status.vd_v = voltage_command.d;
    s_status.vq_v = voltage_command.q;

    foc_alphabeta_t voltage_ab = foc_inverse_park(voltage_command, rotor);
    if (s_dead_time_compensation) {
        /* The currents here are the measured ones, not the references: the
         * compensation is a function of which body diode is conducting, and the
         * drive only learns that from the shunt. */
        voltage_ab = foc_dead_time_compensate(voltage_ab, (foc_phase_t){ia_a, ib_a, ic_a}, bus_v,
                                              s_board->dead_time_s, s_board->pwm_period_s,
                                              FIRMWARE_DEAD_TIME_LINEAR_A);
    }
    const foc_phase_t duty = foc_svpwm(voltage_ab, bus_v);
    (void)hal_pwm_set_duty(duty.a, duty.b, duty.c);
    hal_watchdog_kick();
    s_status.ticks += 1u;
}

const firmware_status_t *firmware_status(void) {
    return &s_status;
}
