/* hal.h, the only header firmware may include besides the C standard library.
 * Two implementations exist: harness/src/sim_hal.cpp on the host, and
 * hal_stm32.c on a real board. Firmware never includes a simulator header and
 * never calls a simulator function.
 *
 * Units are in every name. Frame is stated per call:
 *   phase      a, b, c stator quantities
 *   rotor      mechanical angle and speed of the rotor
 *   electrical electrical angle in rad, dq frame currents
 *   inverter   DC link, gate signals, duty ratios
 */
#ifndef TSIM_HAL_H
#define TSIM_HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Rates are owned by the firmware, so the simulator and a board schedule the
 * same three loops. */
#define HAL_CONTROL_RATE_HZ 20000u /* current loop, Hz */
#define HAL_CONTROL_PERIOD_US 50u  /* 1 / control rate, us */
#define HAL_VELOCITY_DIVIDER 10u   /* velocity loop every 10 ticks, 2 kHz */
#define HAL_POSITION_DIVIDER 20u   /* position loop every 20 ticks, 1 kHz */

typedef enum { HAL_OK = 0, HAL_ERR_ARGUMENT = 1, HAL_ERR_STATE = 2 } hal_status_t;

/* Latched fault bits. The HAL reports, the firmware decides. */
#define HAL_FAULT_OVERCURRENT (1u << 0)  /* phase current above limit, A */
#define HAL_FAULT_OVERVOLTAGE (1u << 1)  /* DC link above limit, V */
#define HAL_FAULT_UNDERVOLTAGE (1u << 2) /* DC link below limit, V */
#define HAL_FAULT_OVERTEMP (1u << 3)     /* power stage above limit, degC */
#define HAL_FAULT_ENCODER_LOSS (1u << 4) /* no valid encoder sample, bool */
#define HAL_FAULT_WATCHDOG (1u << 5)     /* kick missed, bool */
#define HAL_FAULT_GATE_DRIVER (1u << 6)  /* driver fault pin, bool */

/* Board constants. A board fills this with its own values, so firmware source
 * does not change when the hardware changes. */
typedef struct {
    uint32_t pole_pairs;                /* pole pairs, count */
    int32_t encoder_counts_per_rev;     /* quadrature counts per rotor rev, count/rev */
    int32_t encoder_zero_offset_counts; /* rotor frame, count */
    bool encoder_inverted;              /* rotor frame direction, bool */
    float shunt_resistance_ohm;         /* phase shunt, ohm */
    float current_amp_gain_v_per_v;     /* shunt amplifier gain, V/V */
    float adc_reference_v;              /* ADC reference, V */
    uint16_t adc_full_scale_counts;     /* ADC full scale, count */
    float bus_voltage_nominal_v;        /* DC link nominal, V */
    float dead_time_s;                  /* gate dead time, s */
    float pwm_period_s;                 /* switching period, s */
} hal_board_config_t;

/* Boot. Call once before anything else. Returns HAL_OK on success. */
hal_status_t hal_init(void);

/* Board constants for this target. Never NULL after a successful hal_init. */
const hal_board_config_t *hal_board_config(void);

/* Set phase duty ratios in [0, 1], phase frame. 0.5 is 50 percent on time.
 * Values are clipped. Takes effect at the next switching period boundary,
 * like a shadow register. */
hal_status_t hal_pwm_set_duty(float duty_a, float duty_b, float duty_c);

/* Enable or disable all six gate outputs, inverter frame. Disabled means the
 * windings are open. */
void hal_pwm_enable(bool enable);

/* Set gate dead time, ns, inverter frame. Harness hook for the dead time
 * study. A board fixes this at assembly time. */
hal_status_t hal_pwm_set_dead_time_ns(uint32_t dead_time_ns);

typedef struct {
    float ia_a;            /* phase A current, A, phase frame */
    float ib_a;            /* phase B current, A, phase frame */
    float ic_a;            /* phase C current, A, phase frame */
    uint32_t timestamp_us; /* instant the sample and hold latched, us */
    bool valid;            /* conversion finished and within range, bool */
} hal_current_sample_t;

typedef struct {
    int64_t position_counts; /* continuous signed rotor position, count, rotor frame */
    uint32_t timestamp_us;   /* instant the position was valid, us */
    bool index_seen;         /* index pulse observed since boot, bool */
    bool valid;              /* no gap or decode error, bool */
} hal_encoder_sample_t;

/* Last complete phase current sample, A, phase frame. Does not block. */
hal_status_t hal_current_read(hal_current_sample_t *out);

/* Last complete encoder sample, count, rotor frame. Does not block. */
hal_status_t hal_encoder_read(hal_encoder_sample_t *out);

/* DC link voltage, V, inverter frame. Last conversion, does not block. */
float hal_bus_voltage_read_v(void);

/* Power stage temperature, degC, inverter frame. Last conversion. */
float hal_temperature_read_c(void);

/* Latched fault bits as a HAL_FAULT_* mask. Does not clear. */
uint32_t hal_fault_read(void);

/* Clear latched hardware bits. Firmware calls this only after copying them. */
void hal_fault_clear(void);

/* Latch a firmware originated fault code into the same register. */
void hal_fault_latch(uint32_t code);

/* Kick the watchdog. Must be called within the timeout or the board resets. */
void hal_watchdog_kick(void);

/* Watchdog timeout, us. A board fixes this; the harness may shorten it. */
hal_status_t hal_watchdog_set_timeout_us(uint32_t timeout_us);

/* Monotonic microseconds since boot, us. Never decreases. */
uint64_t hal_time_us(void);

/* CPU cycles since boot, cycles. Wraps every 2^32 cycles. WCET only. */
uint32_t hal_cpu_cycles(void);

/* Busy wait, us. Not permitted inside the control tick. */
void hal_delay_us(uint32_t delay_us);

typedef void (*hal_tick_fn_t)(void);

/* Register the 20 kHz control tick. On a board the tick runs in interrupt
 * context, on the host it runs on the harness thread. It must finish inside
 * 50 us. */
hal_status_t hal_set_control_tick(hal_tick_fn_t tick_fn);

/* Start calling the registered tick every 50 us. The harness owns the clock
 * and does not call this. */
hal_status_t hal_start_control_timing(void);

/* Stop the tick. Safe to call from non tick context. */
void hal_stop_control_timing(void);

/* Disable interrupts, return the previous mask. The host build returns 0. */
uint32_t hal_irq_save(void);

/* Restore the mask returned by hal_irq_save. */
void hal_irq_restore(uint32_t mask);

/* Non blocking push of one telemetry frame, bytes. False if the ring is full,
 * so the firmware counts drops instead of waiting. */
bool hal_telemetry_push(const void *frame, size_t length_bytes);

/* Dropped telemetry frames since boot, count. */
uint32_t hal_telemetry_drop_count(void);

/* Status LED, bool. */
void hal_status_led_set(bool on);

/* Latch a panic code and source line. Never returns on a board. */
void hal_panic(uint32_t code, uint32_t line);

#ifdef __cplusplus
}
#endif

#endif /* TSIM_HAL_H */
