/* foc.h, coordinate transforms and modulation. Pure functions of their
 * arguments: no state, no library call. The control tick therefore does not
 * depend on libm, and the portable target links nothing but hal_.
 *
 * Conventions, written once, because a sign error here is the failure this
 * project exists to catch:
 *   amplitude invariant Clarke, so a balanced set of peak amplitude I maps to
 *   a vector of length I in the alpha beta frame
 *   Park: d = alpha cos + beta sin, q = -alpha sin + beta cos
 *   inverse Park: alpha = d cos - q sin, beta = d sin + q cos
 *   positive q current produces positive torque, and positive torque
 *   accelerates positive rotation
 * docs/spec.md states the same equations in prose. The tests assert that
 * forward and inverse agree, and that a balanced set at electrical speed maps
 * to a constant q current, which is the shape of the check a sign error fails.
 */
#ifndef TSIM_FOC_H
#define TSIM_FOC_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float a; /* phase quantity, unit depends on the call */
    float b;
    float c;
} foc_phase_t;

typedef struct {
    float alpha; /* stationary frame, unit depends on the call */
    float beta;
} foc_alphabeta_t;

typedef struct {
    float d; /* rotor frame, unit depends on the call */
    float q;
} foc_dq_t;

typedef struct {
    float sin_theta; /* sine and cosine of the electrical angle, unitless */
    float cos_theta;
} foc_rotor_t;

/* Sine of an angle in rad, maximum absolute error 1.5e-7 on [-pi, pi).
 * Eleventh order Taylor series after a quadrant fold. No libm. */
float foc_sin(float theta_rad);

/* Cosine with the same error bound. */
float foc_cos(float theta_rad);

/* Square root of a non negative value. The argument is scaled into [1, 4) with
 * exact power of two steps, then six Newton iterations run. Returns 0 for a
 * non positive input. Relative error below 1e-6 and fixed work, no libm. */
float foc_sqrt(float x);

/* Sine and cosine of the electrical angle, computed once per tick. */
foc_rotor_t foc_rotor(float theta_e_rad);

/* Phase to alpha beta, amplitude invariant. A common mode offset in the three
 * phases is removed, which is what a floating wye neutral does. */
foc_alphabeta_t foc_clarke(foc_phase_t phase);

/* Alpha beta back to the three phases, the inverse of the Clarke above. */
foc_phase_t foc_inverse_clarke(foc_alphabeta_t ab);

foc_dq_t foc_park(foc_alphabeta_t ab, foc_rotor_t rotor);

foc_alphabeta_t foc_inverse_park(foc_dq_t dq, foc_rotor_t rotor);

/* Scale a dq vector into the circle of radius limit_v, direction kept. A
 * limit of zero or less returns a zero vector. */
foc_dq_t foc_limit_magnitude(foc_dq_t dq, float limit_v);

/* Phase voltages to duty ratios in [0, 1] for a measured bus voltage. Minimum
 * max injection: the zero sequence is chosen so the largest duty excursion is
 * as small as it can be, which is the extra 15 percent of linear range that
 * space vector modulation has over sine triangle modulation. */
foc_phase_t foc_svpwm(foc_alphabeta_t voltage_ab_v, float bus_v);

/* Add back the volt seconds the gate dead time removes, phase frame.
 *
 * Each leg loses bus * dead_time / period of average voltage while its current
 * is positive and gains the same while it is negative, because during the dead
 * interval the winding freewheels through whichever body diode is forward
 * biased. That is a voltage error of the sign of the current, and left alone it
 * is a 0.36 V square wave at 36 V and 500 ns, which at this winding is more
 * than an amp of distortion.
 *
 * The compensation is the same expression with the opposite sign, added to
 * each phase voltage and then referred back to alpha beta with the common mode
 * removed, since the floating neutral cannot see it. linear_a fades the
 * correction out below the current where the sign stops being single valued
 * inside one switching period; passing zero gives a hard sign.
 *
 * Returns the input unchanged when the bus, the dead time or the period is not
 * positive, so a drive that has not been commissioned cannot divide by zero. */
foc_alphabeta_t foc_dead_time_compensate(foc_alphabeta_t voltage_ab_v, foc_phase_t current_a,
                                         float bus_v, float dead_time_s, float pwm_period_s,
                                         float linear_a);

#ifdef __cplusplus
}
#endif

#endif /* TSIM_FOC_H */
