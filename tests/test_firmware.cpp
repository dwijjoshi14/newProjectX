#include "firmware.h"
#include "stub_hal_probe.h"
#include "tsim_test.h"

/* The thermal limit is 155 degC of insulation and the drive acts 25 K below it.
 * These cases move the temperature sensor instead of running a six second
 * simulation, because the thing being tested is the arithmetic on the number the
 * sensor returns, and a soak test would be testing the plant instead. */
TSIM_TEST(firmware_holds_the_stall_envelope_at_a_bench_temperature) {
    hal_stub_set_temperature_c(25.0f);
    firmware_init();
    TSIM_CHECK_NEAR(firmware_status()->thermal_limit_a, 61.8, 1.0e-3);
    for (int i = 0; i < 200; ++i) {
        firmware_tick();
    }
    TSIM_CHECK_NEAR(firmware_status()->thermal_limit_a, 61.8, 1.0e-3);
    TSIM_CHECK(firmware_status()->fault_bits == 0u);
}

TSIM_TEST(firmware_derates_the_envelope_between_the_two_temperatures) {
    hal_stub_set_temperature_c(115.0f);
    firmware_init();
    for (int i = 0; i < 200; ++i) {
        firmware_tick();
    }
    /* Halfway between 100 and 130 degC is half the stall current. */
    TSIM_CHECK_NEAR(firmware_status()->thermal_limit_a, 30.9, 0.2);
    TSIM_CHECK(firmware_status()->fault_bits == 0u);

    hal_stub_set_temperature_c(125.0f);
    for (int i = 0; i < 200; ++i) {
        firmware_tick();
    }
    TSIM_CHECK_NEAR(firmware_status()->thermal_limit_a, 10.3, 0.2);
}

TSIM_TEST(firmware_opens_the_gates_at_the_winding_limit) {
    hal_stub_set_temperature_c(131.0f);
    firmware_init();
    for (int i = 0; i < 200; ++i) {
        firmware_tick();
    }
    TSIM_CHECK((firmware_status()->fault_bits & HAL_FAULT_OVERTEMP) != 0u);
    TSIM_CHECK(!hal_stub_pwm_enabled());
}

TSIM_TEST(firmware_tick_runs_against_stub_hal) {
    /* The stub's temperature is process state, like a sensor is board state.
     * A test that runs after one of the thermal cases inherits whatever it left,
     * and at 131 degC the boot latch would open the gates before the first tick
     * and this test would fail for a reason that has nothing to do with it. */
    hal_stub_set_temperature_c(25.0f);
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
