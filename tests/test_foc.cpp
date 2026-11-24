#include <cmath>

#include "foc.h"
#include "tsim_test.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 6.283185307179586;
constexpr double kTwoPiOverThree = 2.0943951023931953;

double larger(double a, double b) {
    return (a > b) ? a : b;
}

} // namespace

TSIM_TEST(foc_sine_and_cosine_match_the_reference_functions) {
    double worst_sin = 0.0;
    double worst_cos = 0.0;
    for (int i = 0; i <= 2048; ++i) {
        const double theta = -10.0 + 20.0 * static_cast<double>(i) / 2048.0;
        const double actual_sin = static_cast<double>(foc_sin(static_cast<float>(theta)));
        const double actual_cos = static_cast<double>(foc_cos(static_cast<float>(theta)));
        worst_sin = larger(worst_sin, std::fabs(actual_sin - std::sin(theta)));
        worst_cos = larger(worst_cos, std::fabs(actual_cos - std::cos(theta)));
    }
    TSIM_CHECK(worst_sin < 1.0e-6);
    TSIM_CHECK(worst_cos < 1.0e-6);

    /* Quadrant edges are where a fold mistake shows up as a sign, not as a
     * small error, so they get their own checks. */
    TSIM_CHECK(std::fabs(static_cast<double>(foc_sin(static_cast<float>(kPi * 0.5))) - 1.0) <
               1.0e-6);
    TSIM_CHECK(std::fabs(static_cast<double>(foc_sin(static_cast<float>(-kPi * 0.5))) + 1.0) <
               1.0e-6);
    TSIM_CHECK(std::fabs(static_cast<double>(foc_sin(static_cast<float>(kPi)))) < 1.0e-6);
    TSIM_CHECK(std::fabs(static_cast<double>(foc_cos(0.0f)) - 1.0) < 1.0e-6);
    TSIM_CHECK(std::fabs(static_cast<double>(foc_cos(static_cast<float>(kPi))) + 1.0) < 1.0e-6);
    TSIM_CHECK(foc_sin(0.0f) == 0.0f);
}

TSIM_TEST(foc_sqrt_is_accurate_across_the_range_the_loop_needs) {
    double worst_relative = 0.0;
    for (int i = 1; i <= 1000; ++i) {
        /* 1e-6 to 1e3, which brackets a squared voltage twice over. */
        const double value = std::pow(10.0, -6.0 + 9.0 * static_cast<double>(i) / 1000.0);
        const double expected = std::sqrt(value);
        const double actual = static_cast<double>(foc_sqrt(static_cast<float>(value)));
        worst_relative = larger(worst_relative, std::fabs(actual - expected) / expected);
    }
    TSIM_CHECK(worst_relative < 1.0e-5);
    TSIM_CHECK(foc_sqrt(0.0f) == 0.0f);
    TSIM_CHECK(foc_sqrt(-1.0f) == 0.0f);
}

TSIM_TEST(clarke_round_trip_recovers_the_phase_quantities) {
    double worst = 0.0;
    for (int i = 0; i < 360; ++i) {
        const double angle = kTwoPi * static_cast<double>(i) / 360.0;
        const double amplitude = 1.0 + 0.5 * std::sin(3.0 * angle);
        foc_phase_t phase;
        phase.a = static_cast<float>(amplitude * std::cos(angle));
        phase.b = static_cast<float>(amplitude * std::cos(angle - kTwoPiOverThree));
        phase.c = static_cast<float>(amplitude * std::cos(angle + kTwoPiOverThree));

        const foc_phase_t back = foc_inverse_clarke(foc_clarke(phase));
        worst = larger(worst, std::fabs(static_cast<double>(back.a) - phase.a));
        worst = larger(worst, std::fabs(static_cast<double>(back.b) - phase.b));
        worst = larger(worst, std::fabs(static_cast<double>(back.c) - phase.c));
    }
    TSIM_CHECK(worst < 1.0e-5);
}

TSIM_TEST(clarke_removes_a_common_mode_offset) {
    foc_phase_t phase;
    phase.a = 1.0f;
    phase.b = -0.5f;
    phase.c = -0.5f;
    const foc_alphabeta_t reference = foc_clarke(phase);

    phase.a += 7.0f;
    phase.b += 7.0f;
    phase.c += 7.0f;
    const foc_alphabeta_t shifted = foc_clarke(phase);

    TSIM_CHECK_NEAR(shifted.alpha, reference.alpha, 1.0e-6);
    TSIM_CHECK_NEAR(shifted.beta, reference.beta, 1.0e-6);
}

TSIM_TEST(park_and_inverse_park_round_trip) {
    double worst = 0.0;
    for (int i = 0; i < 180; ++i) {
        const double theta = kTwoPi * static_cast<double>(i) / 180.0;
        const foc_rotor_t rotor = foc_rotor(static_cast<float>(theta));
        for (int j = 0; j < 8; ++j) {
            foc_dq_t requested;
            requested.d = static_cast<float>(0.5 * std::cos(0.7 * j) + 1.0);
            requested.q = static_cast<float>(2.0 * std::sin(1.1 * j));
            const foc_dq_t back = foc_park(foc_inverse_park(requested, rotor), rotor);
            worst = larger(worst, std::fabs(static_cast<double>(back.d) - requested.d));
            worst = larger(worst, std::fabs(static_cast<double>(back.q) - requested.q));
        }
    }
    TSIM_CHECK(worst < 1.0e-5);
}

TSIM_TEST(a_balanced_set_at_electrical_speed_maps_to_a_constant_q_current) {
    /* Build the phase currents a real machine would carry at a given rotor
     * angle, with the current vector on the q axis. Any sign or frame error in
     * the Park transform turns this into a rotating or negated q current. */
    const double amplitude_a = 2.0;
    double worst_id = 0.0;
    double worst_iq = 0.0;
    for (int i = 0; i < 360; ++i) {
        const double theta_e = kTwoPi * static_cast<double>(i) / 360.0;
        const double phi = theta_e + kPi * 0.5;
        foc_phase_t phase;
        phase.a = static_cast<float>(amplitude_a * std::cos(phi));
        phase.b = static_cast<float>(amplitude_a * std::cos(phi - kTwoPiOverThree));
        phase.c = static_cast<float>(amplitude_a * std::cos(phi + kTwoPiOverThree));

        const foc_dq_t dq = foc_park(foc_clarke(phase), foc_rotor(static_cast<float>(theta_e)));
        worst_id = larger(worst_id, std::fabs(static_cast<double>(dq.d)));
        worst_iq = larger(worst_iq, std::fabs(static_cast<double>(dq.q) - amplitude_a));
    }
    TSIM_CHECK(worst_id < 1.0e-4);
    TSIM_CHECK(worst_iq < 1.0e-4);
}

TSIM_TEST(svpwm_keeps_the_differential_voltage_and_the_duty_in_range) {
    const double bus_v = 36.0;
    const double magnitude_v = 18.0; /* inside 0.95 * bus / sqrt(3) = 19.74 V */
    double worst_voltage = 0.0;
    double lowest_duty = 1.0;
    double highest_duty = 0.0;

    for (int i = 0; i < 720; ++i) {
        const double angle = kTwoPi * static_cast<double>(i) / 720.0;
        foc_alphabeta_t requested;
        requested.alpha = static_cast<float>(magnitude_v * std::cos(angle));
        requested.beta = static_cast<float>(magnitude_v * std::sin(angle));

        const foc_phase_t duty = foc_svpwm(requested, static_cast<float>(bus_v));
        lowest_duty = (duty.a < lowest_duty) ? duty.a : lowest_duty;
        lowest_duty = (duty.b < lowest_duty) ? duty.b : lowest_duty;
        lowest_duty = (duty.c < lowest_duty) ? duty.c : lowest_duty;
        highest_duty = (duty.a > highest_duty) ? duty.a : highest_duty;
        highest_duty = (duty.b > highest_duty) ? duty.b : highest_duty;
        highest_duty = (duty.c > highest_duty) ? duty.c : highest_duty;

        /* The plant removes the common mode, so reconstruct the phase voltages
         * the winding actually sees and compare against what was asked for. */
        const double mean_duty = (static_cast<double>(duty.a) + duty.b + duty.c) / 3.0;
        foc_phase_t realised;
        realised.a = static_cast<float>(bus_v * (static_cast<double>(duty.a) - mean_duty));
        realised.b = static_cast<float>(bus_v * (static_cast<double>(duty.b) - mean_duty));
        realised.c = static_cast<float>(bus_v * (static_cast<double>(duty.c) - mean_duty));

        const foc_phase_t expected = foc_inverse_clarke(requested);
        worst_voltage =
            larger(worst_voltage, std::fabs(static_cast<double>(realised.a) - expected.a));
        worst_voltage =
            larger(worst_voltage, std::fabs(static_cast<double>(realised.b) - expected.b));
        worst_voltage =
            larger(worst_voltage, std::fabs(static_cast<double>(realised.c) - expected.c));
    }

    TSIM_CHECK(worst_voltage < 1.0e-4);
    TSIM_CHECK(lowest_duty >= 0.0f);
    TSIM_CHECK(highest_duty <= 1.0f);
    /* Minimum max injection puts the extreme duty at 0.5 + 0.5 * sqrt(3)/2 *
     * V / bus, which for 18 V is 0.933. Anything near 1.0 would mean the offset
     * is not being applied at all. */
    TSIM_CHECK(highest_duty < 0.95f);
}

TSIM_TEST(svpwm_clamps_an_over_modulated_request) {
    foc_alphabeta_t requested;
    requested.alpha = 40.0f; /* well past the 20.8 V linear limit at 36 V */
    requested.beta = 0.0f;
    const foc_phase_t duty = foc_svpwm(requested, 36.0f);
    TSIM_CHECK(duty.a <= 1.0f);
    TSIM_CHECK(duty.a >= 0.0f);
    TSIM_CHECK(duty.b <= 1.0f);
    TSIM_CHECK(duty.b >= 0.0f);
    TSIM_CHECK(duty.c <= 1.0f);
    TSIM_CHECK(duty.c >= 0.0f);
    /* A zero bus is a dead bus: the safe answer is the zero vector. */
    const foc_phase_t dead = foc_svpwm(requested, 0.0f);
    TSIM_CHECK(dead.a == 0.5f);
    TSIM_CHECK(dead.b == 0.5f);
    TSIM_CHECK(dead.c == 0.5f);
}

TSIM_TEST(dead_time_compensation_is_the_inverse_of_the_voltage_the_gates_lose) {
    /* Bus 36 V, 500 ns in a 50 us period: 0.36 V per leg, signed by the phase
     * current. The currents sum to zero as they must in a floating wye, so the
     * raw correction has a common mode of -0.12 V which has to be removed.
     * What is left is a 0.48 V move on alpha, not 0.36 V, and that difference
     * is the neutral shift rather than an error. */
    foc_alphabeta_t requested;
    requested.alpha = 3.0f;
    requested.beta = 0.0f;
    foc_phase_t current_a;
    current_a.a = 2.0f;
    current_a.b = -1.0f;
    current_a.c = -1.0f;

    const foc_alphabeta_t corrected =
        foc_dead_time_compensate(requested, current_a, 36.0f, 500.0e-9f, 50.0e-6f, 0.5f);
    TSIM_CHECK_NEAR(corrected.alpha, 3.48, 1.0e-4);
    TSIM_CHECK_NEAR(corrected.beta, 0.0, 1.0e-4);

    /* The inverse sign for the negative half of the cycle. */
    current_a.a = -2.0f;
    current_a.b = 1.0f;
    current_a.c = 1.0f;
    const foc_alphabeta_t mirrored =
        foc_dead_time_compensate(requested, current_a, 36.0f, 500.0e-9f, 50.0e-6f, 0.5f);
    TSIM_CHECK_NEAR(mirrored.alpha, 2.52, 1.0e-4);

    /* Three equal currents: the correction is pure common mode, and a floating
     * neutral cannot see it. A drive that applied the correction without
     * removing the mean would push its own neutral around and call it a fix. */
    current_a.a = 2.0f;
    current_a.b = 2.0f;
    current_a.c = 2.0f;
    const foc_alphabeta_t common =
        foc_dead_time_compensate(requested, current_a, 36.0f, 500.0e-9f, 50.0e-6f, 0.5f);
    TSIM_CHECK_NEAR(common.alpha, 3.0, 1.0e-5);
    TSIM_CHECK_NEAR(common.beta, 0.0, 1.0e-5);

    /* Zero current, or a drive that has not been commissioned, changes
     * nothing. Both of those are the states a real board boots in. */
    current_a.a = 0.0f;
    current_a.b = 0.0f;
    current_a.c = 0.0f;
    const foc_alphabeta_t no_current =
        foc_dead_time_compensate(requested, current_a, 36.0f, 500.0e-9f, 50.0e-6f, 0.5f);
    TSIM_CHECK_NEAR(no_current.alpha, 3.0, 1.0e-5);
    const foc_alphabeta_t no_dead_time =
        foc_dead_time_compensate(requested, current_a, 36.0f, 0.0f, 50.0e-6f, 0.5f);
    TSIM_CHECK_NEAR(no_dead_time.alpha, 3.0, 1.0e-5);
    const foc_alphabeta_t no_bus =
        foc_dead_time_compensate(requested, current_a, 0.0f, 500.0e-9f, 50.0e-6f, 0.5f);
    TSIM_CHECK_NEAR(no_bus.alpha, 3.0, 1.0e-5);
}

TSIM_TEST(vector_limiting_scales_the_direction_into_the_circle) {
    foc_dq_t vector;
    vector.d = 30.0f;
    vector.q = 40.0f; /* magnitude 50 */
    const foc_dq_t limited = foc_limit_magnitude(vector, 19.74f);
    const double magnitude = std::sqrt(static_cast<double>(limited.d) * limited.d +
                                       static_cast<double>(limited.q) * limited.q);
    TSIM_CHECK_NEAR(magnitude, 19.74, 1.0e-4);
    TSIM_CHECK_NEAR(limited.q / limited.d, 40.0 / 30.0, 1.0e-5);

    const foc_dq_t untouched = foc_limit_magnitude(vector, 100.0f);
    TSIM_CHECK(untouched.d == 30.0f);
    TSIM_CHECK(untouched.q == 40.0f);
}
