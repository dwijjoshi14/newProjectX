/* pi.h, one proportional plus integral regulator with clamping anti windup.
 *
 * The integrator is not a free accumulator. Whenever the output would leave
 * the limit, the integral is pulled back so that proportional plus integral
 * equals the limit exactly. That is back calculation without the extra gain:
 * it stops the winding up that a saturating current reference causes, and it
 * leaves the loop untouched while the output is inside the limit.
 */
#ifndef TSIM_PI_H
#define TSIM_PI_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;          /* proportional gain, output unit per error unit */
    float ki_times_ts; /* integral gain times the period, output unit per error unit */
    float limit;       /* symmetric output limit, output unit */
    float integral;    /* integrator state, output unit */
} pi_regulator_t;

/* kp and ki have the usual units, period_s is the loop period, limit is the
 * symmetric clamp on the output. */
void pi_init(pi_regulator_t *regulator, float kp, float ki, float period_s, float limit);

/* Zero the integrator, keep the gains. */
void pi_reset(pi_regulator_t *regulator);

/* Change the output limit, for a voltage limit that follows the bus. */
void pi_set_limit(pi_regulator_t *regulator, float limit);

/* One update. Returns the clamped output. */
float pi_step(pi_regulator_t *regulator, float error);

#ifdef __cplusplus
}
#endif

#endif /* TSIM_PI_H */
