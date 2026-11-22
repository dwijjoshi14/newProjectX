/* firmware.h, the drive entry points. Units in names, frames per field. */
#ifndef TSIM_FIRMWARE_H
#define TSIM_FIRMWARE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t ticks;       /* control ticks executed since boot, count */
    float theta_e_rad;    /* electrical angle, rad, electrical frame */
    float current_peak_a; /* largest phase current magnitude in the last tick, A */
    uint32_t fault_bits;  /* last latched fault mask, HAL_FAULT_* bits */
} firmware_status_t;

/* Bring the drive up: HAL, sensor offsets, PWM off then armed. */
void firmware_init(void);

/* Start the 20 kHz tick. The harness does not call this, it calls the tick. */
void firmware_start(void);

/* One control tick, 20 kHz. Must finish inside 50 us. */
void firmware_tick(void);

/* Latest status snapshot. Never NULL after firmware_init. */
const firmware_status_t *firmware_status(void);

#ifdef __cplusplus
}
#endif

#endif /* TSIM_FIRMWARE_H */
