#include "pi.h"

void pi_init(pi_regulator_t *regulator, float kp, float ki, float period_s, float limit) {
    regulator->kp = kp;
    regulator->ki_times_ts = ki * period_s;
    regulator->limit = limit;
    regulator->integral = 0.0f;
}

void pi_reset(pi_regulator_t *regulator) {
    regulator->integral = 0.0f;
}

void pi_set_limit(pi_regulator_t *regulator, float limit) {
    regulator->limit = limit;
}

float pi_step(pi_regulator_t *regulator, float error) {
    const float proportional = regulator->kp * error;
    float integral = regulator->integral + regulator->ki_times_ts * error;
    float output = proportional + integral;

    if (output > regulator->limit) {
        output = regulator->limit;
        integral = regulator->limit - proportional;
    } else if (output < -regulator->limit) {
        output = -regulator->limit;
        integral = -regulator->limit - proportional;
    }

    /* A proportional term larger than the limit would otherwise pull the
     * integral outside it and the next update would start from there. */
    if (integral > regulator->limit) {
        integral = regulator->limit;
    } else if (integral < -regulator->limit) {
        integral = -regulator->limit;
    }

    regulator->integral = integral;
    return output;
}
