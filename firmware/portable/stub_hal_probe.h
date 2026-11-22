/* Probes for tests that run firmware against the stub HAL. Test only, the
 * firmware itself never sees this header. */
#ifndef TSIM_STUB_HAL_PROBE_H
#define TSIM_STUB_HAL_PROBE_H

#include <stdbool.h>
#include <stdint.h>

#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Duty written for one phase, duty. 0 to 2, out of range returns 0. */
float hal_stub_last_duty(uint32_t phase_index);

/* True when the gate outputs are enabled, bool. */
bool hal_stub_pwm_enabled(void);

/* Dead time last written, ns. */
uint32_t hal_stub_dead_time_ns(void);

/* True when hal_start_control_timing has been called, bool. */
bool hal_stub_timing_running(void);

/* Tick function registered by the firmware, may be NULL. */
hal_tick_fn_t hal_stub_tick_fn(void);

#ifdef __cplusplus
}
#endif

#endif /* TSIM_STUB_HAL_PROBE_H */
