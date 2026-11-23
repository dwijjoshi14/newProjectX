/* firmware.h, the drive entry points. Units in names, frames per field. */
#ifndef TSIM_FIRMWARE_H
#define TSIM_FIRMWARE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* What the reference path is doing. A mode change zeroes both integrators,
 * because carrying an integrator across a reference step is how a loop winds
 * up before its first tick. */
typedef enum {
    FIRMWARE_MODE_OFF = 0,     /* gates on, zero vector, no torque */
    FIRMWARE_MODE_CURRENT = 1, /* q axis current reference, A */
    FIRMWARE_MODE_SPEED = 2,   /* rotor speed reference, rad/s */
} firmware_mode_t;

typedef struct {
    uint32_t ticks;       /* control ticks executed since boot, count */
    firmware_mode_t mode; /* active reference path */
    float theta_e_rad;    /* electrical angle, rad, electrical frame */
    float current_peak_a; /* largest phase current magnitude in the last tick, A */
    float omega_rad_s;    /* measured rotor speed, rad/s, rotor frame */
    float iq_ref_a;       /* q axis current reference in use, A */
    float id_a;           /* measured d axis current, A, rotor frame */
    float iq_a;           /* measured q axis current, A, rotor frame */
    float vd_v;           /* voltage applied, d axis, V */
    float vq_v;           /* voltage applied, q axis, V */
    float bus_v;          /* DC link measured this tick, V */
    uint32_t fault_bits;  /* last latched fault mask, HAL_FAULT_* bits */
} firmware_status_t;

/* Bring the drive up: HAL, sensor offsets, PWM off then armed. */
void firmware_init(void);

/* Start the 20 kHz tick. The harness does not call this, it calls the tick. */
void firmware_start(void);

/* Choose the reference path and clear the integrators. */
void firmware_set_mode(firmware_mode_t mode);

/* q axis current reference, A. Used in current mode. Positive produces positive
 * torque. Clipped to the continuous current rating. */
void firmware_set_current_reference_a(float iq_a);

/* Rotor speed reference, rad/s, rotor frame. Used in speed mode. */
void firmware_set_speed_reference_rad_s(float omega_rad_s);

/* One control tick, 20 kHz. Must finish inside 50 us. */
void firmware_tick(void);

/* Latest status snapshot. Never NULL after firmware_init. */
const firmware_status_t *firmware_status(void);

#ifdef __cplusplus
}
#endif

#endif /* TSIM_FIRMWARE_H */
