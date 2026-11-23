#include "foc.h"

#include <stdint.h>

#define FOC_TWO_PI_RAD 6.28318548f     /* 2 pi, rad */
#define FOC_PI_RAD 3.14159274f         /* pi, rad */
#define FOC_HALF_PI_RAD 1.57079637f    /* pi / 2, rad */
#define FOC_INV_TWO_PI 0.15915494f     /* 1 / (2 pi), unitless */
#define FOC_ONE_OVER_SQRT3 0.57735027f /* 1 / sqrt(3), unitless */
#define FOC_SQRT3_OVER_TWO 0.86602540f /* sqrt(3) / 2, unitless */
#define FOC_SQRT_SCALINGS 12u          /* power of two steps into [1, 4) */
#define FOC_SQRT_ITERATIONS 6u         /* Newton steps for the square root */

/* Taylor coefficients of sin, x - x^3/3! + x^5/5! - ... + x^11/11!. On
 * [-pi/2, pi/2] the first omitted term is bounded by (pi/2)^13 / 13! = 1.4e-7,
 * which is the number quoted in foc.h. A lookup table would be faster on a
 * part without an FPU and larger in flash; this build is not flash bound. */
#define FOC_SIN_C3 (-1.66666672e-1f)  /* -1/6 */
#define FOC_SIN_C5 (8.33333377e-3f)   /* 1/120 */
#define FOC_SIN_C7 (-1.98412698e-4f)  /* -1/5040 */
#define FOC_SIN_C9 (2.75573192e-6f)   /* 1/362880 */
#define FOC_SIN_C11 (-2.50521084e-8f) /* -1/39916800 */

/* Reduce any angle to [-pi, pi). Callers on the control path wrap first, so
 * the cast below never sees a value large enough to overflow. */
static float foc_reduce_to_pi(float angle_rad) {
    const float turns = (float)((int32_t)(angle_rad * FOC_INV_TWO_PI));
    float x = angle_rad - turns * FOC_TWO_PI_RAD;
    if (x >= FOC_PI_RAD) {
        x -= FOC_TWO_PI_RAD;
    }
    if (x < -FOC_PI_RAD) {
        x += FOC_TWO_PI_RAD;
    }
    return x;
}

float foc_sin(float theta_rad) {
    float x = foc_reduce_to_pi(theta_rad);
    if (x > FOC_HALF_PI_RAD) {
        x = FOC_PI_RAD - x;
    } else if (x < -FOC_HALF_PI_RAD) {
        x = -FOC_PI_RAD - x;
    }
    const float x2 = x * x;
    return x * (1.0f + x2 * (FOC_SIN_C3 +
                             x2 * (FOC_SIN_C5 +
                                   x2 * (FOC_SIN_C7 + x2 * (FOC_SIN_C9 + x2 * FOC_SIN_C11)))));
}

float foc_cos(float theta_rad) {
    return foc_sin(theta_rad + FOC_HALF_PI_RAD);
}

float foc_sqrt(float x) {
    if (!(x > 0.0f)) {
        return 0.0f;
    }
    /* Normalise into [1, 4) with quartering or quadrupling, both exact powers
     * of two, so the Newton seed is within a factor of two of the root. Newton
     * from a far guess crawls: starting at 1 for an argument of 1e-6 needs ten
     * steps just to arrive. The scaling loop is fixed length, which keeps the
     * work bounded, and 12 steps cover 1.5e-8 to 6.7e7, far outside anything a
     * 36 V bus can ask for. */
    float scaled = x;
    float factor = 1.0f;
    for (uint32_t i = 0u; i < FOC_SQRT_SCALINGS; ++i) {
        if (scaled > 4.0f) {
            scaled *= 0.25f;
            factor *= 2.0f;
        } else if (scaled < 1.0f) {
            scaled *= 4.0f;
            factor *= 0.5f;
        }
    }
    float guess = scaled;
    for (uint32_t i = 0u; i < FOC_SQRT_ITERATIONS; ++i) {
        guess = 0.5f * (guess + scaled / guess);
    }
    return guess * factor;
}

foc_rotor_t foc_rotor(float theta_e_rad) {
    foc_rotor_t rotor;
    rotor.sin_theta = foc_sin(theta_e_rad);
    rotor.cos_theta = foc_cos(theta_e_rad);
    return rotor;
}

foc_alphabeta_t foc_clarke(foc_phase_t phase) {
    foc_alphabeta_t ab;
    ab.alpha = (2.0f / 3.0f) * (phase.a - 0.5f * phase.b - 0.5f * phase.c);
    ab.beta = FOC_ONE_OVER_SQRT3 * (phase.b - phase.c);
    return ab;
}

foc_phase_t foc_inverse_clarke(foc_alphabeta_t ab) {
    foc_phase_t phase;
    phase.a = ab.alpha;
    phase.b = -0.5f * ab.alpha + FOC_SQRT3_OVER_TWO * ab.beta;
    phase.c = -0.5f * ab.alpha - FOC_SQRT3_OVER_TWO * ab.beta;
    return phase;
}

foc_dq_t foc_park(foc_alphabeta_t ab, foc_rotor_t rotor) {
    foc_dq_t dq;
    dq.d = ab.alpha * rotor.cos_theta + ab.beta * rotor.sin_theta;
    dq.q = -ab.alpha * rotor.sin_theta + ab.beta * rotor.cos_theta;
    return dq;
}

foc_alphabeta_t foc_inverse_park(foc_dq_t dq, foc_rotor_t rotor) {
    foc_alphabeta_t ab;
    ab.alpha = dq.d * rotor.cos_theta - dq.q * rotor.sin_theta;
    ab.beta = dq.d * rotor.sin_theta + dq.q * rotor.cos_theta;
    return ab;
}

foc_dq_t foc_limit_magnitude(foc_dq_t dq, float limit_v) {
    if (!(limit_v > 0.0f)) {
        dq.d = 0.0f;
        dq.q = 0.0f;
        return dq;
    }
    const float magnitude_v = foc_sqrt(dq.d * dq.d + dq.q * dq.q);
    if (magnitude_v > limit_v) {
        const float scale = limit_v / magnitude_v;
        dq.d *= scale;
        dq.q *= scale;
    }
    return dq;
}

/* Clamp a duty to the unit interval. A modulation that asks for more than the
 * bus can deliver is limited here rather than rejected by the HAL, so the gate
 * driver always receives three valid ratios. */
static float foc_clamp_duty(float duty) {
    if (duty < 0.0f) {
        return 0.0f;
    }
    if (duty > 1.0f) {
        return 1.0f;
    }
    return duty;
}

foc_phase_t foc_svpwm(foc_alphabeta_t voltage_ab_v, float bus_v) {
    foc_phase_t duty;
    if (!(bus_v > 0.0f)) {
        duty.a = 0.5f;
        duty.b = 0.5f;
        duty.c = 0.5f;
        return duty;
    }

    const float v_a = voltage_ab_v.alpha;
    const float v_b = -0.5f * voltage_ab_v.alpha + FOC_SQRT3_OVER_TWO * voltage_ab_v.beta;
    const float v_c = -0.5f * voltage_ab_v.alpha - FOC_SQRT3_OVER_TWO * voltage_ab_v.beta;

    float v_max = v_a;
    if (v_b > v_max) {
        v_max = v_b;
    }
    if (v_c > v_max) {
        v_max = v_c;
    }
    float v_min = v_a;
    if (v_b < v_min) {
        v_min = v_b;
    }
    if (v_c < v_min) {
        v_min = v_c;
    }

    /* Half the sum is pushed onto all three phases, so the differential
     * voltages are untouched and the extreme duty sits as close to 0.5 as it
     * can. The plant removes this common mode the way a floating neutral does,
     * so the injection is invisible to the average model: its job is range,
     * not voltage. */
    const float offset_v = -0.5f * (v_max + v_min);
    const float inv_bus = 1.0f / bus_v;
    duty.a = foc_clamp_duty(0.5f + (v_a + offset_v) * inv_bus);
    duty.b = foc_clamp_duty(0.5f + (v_b + offset_v) * inv_bus);
    duty.c = foc_clamp_duty(0.5f + (v_c + offset_v) * inv_bus);
    return duty;
}
