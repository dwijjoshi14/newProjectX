#include "firmware.h"
#include "stub_hal_probe.h"
#include "tsim_test.h"

TSIM_TEST(firmware_tick_runs_against_stub_hal) {
    firmware_init();
    for (int i = 0; i < 100; ++i) {
        firmware_tick();
    }

    TSIM_CHECK(firmware_status()->ticks == 100u);
    TSIM_CHECK(firmware_status()->fault_bits == 0u);
    TSIM_CHECK_NEAR(firmware_status()->theta_e_rad, 0.0, 1.0e-9);
    TSIM_CHECK(hal_stub_pwm_enabled());
    TSIM_CHECK_NEAR(hal_stub_last_duty(0), 0.5, 1.0e-6);
    TSIM_CHECK_NEAR(hal_stub_last_duty(1), 0.5, 1.0e-6);
    TSIM_CHECK_NEAR(hal_stub_last_duty(2), 0.5, 1.0e-6);
}

TSIM_TEST(firmware_registers_a_tick_but_does_not_start_its_own_clock) {
    firmware_init();
    const hal_tick_fn_t tick_fn = hal_stub_tick_fn();
    TSIM_CHECK(tick_fn != nullptr);
    TSIM_CHECK(!hal_stub_timing_running());
}
