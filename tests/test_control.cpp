#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "firmware.h"
#include "hal.h"
#include "sim_hal.h"
#include "sim_loop.h"
#include "tsim/ideal_plant.h"
#include "tsim/plant.h"
#include "tsim/plant_params.h"
#include "tsim/switching_plant.h"
#include "tsim_test.h"

namespace {

constexpr std::uint32_t kCurrentLoopTicks = 120; /* 6 ms at 20 kHz */
constexpr std::uint32_t kSpeedLoopTicks = 6000;  /* 300 ms at 20 kHz */
constexpr std::uint32_t kLoadStepTick = 3000;    /* 150 ms */
constexpr double kLoadTorqueNm = 0.05;
constexpr float kSpeedReferenceRadS = 200.0f;
constexpr float kCurrentReferenceA = 2.0f;

/* Time the signal first reaches a level, interpolated between samples. */
double crossing_time_s(const std::vector<double> &values, double level, double period_s) {
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (values[i] >= level) {
            if (i == 0) {
                return 0.0;
            }
            const double previous = values[i - 1];
            const double rise = values[i] - previous;
            if (rise <= 0.0) {
                return static_cast<double>(i) * period_s;
            }
            const double fraction = (level - previous) / rise;
            return (static_cast<double>(i) - 1.0 + fraction) * period_s;
        }
    }
    return -1.0;
}

/* Plant and inverter for the day 3 measurements: real switching, dead time,
 * device drop, a link that moves. The average plant stays available for the
 * tests that are about the controller alone. */
tsim::InverterParams realistic_inverter() {
    return tsim::inverter_params_realistic();
}

void prepare_locked_plant(tsim::IdealPlant &plant) {
    plant.reset(0.0);
    plant.set_rotor_locked(true);
    sim_hal_attach(plant);
    firmware_init();
    firmware_set_mode(FIRMWARE_MODE_CURRENT);
}

} // namespace

TSIM_TEST(the_harness_latches_the_adc_at_the_window_centre) {
    tsim::IdealPlant plant(tsim::maxon_ec_i_40_70w_params());
    plant.reset(0.0);
    sim_hal_attach(plant);
    firmware_init();

    for (std::uint64_t tick = 0; tick < 20; ++tick) {
        sim_run_window(plant, tick);
        hal_current_sample_t sample;
        TSIM_CHECK(hal_current_read(&sample) == HAL_OK);
        const std::uint32_t expected_us = static_cast<std::uint32_t>(
            tick * tsim::kControlPeriodNs / 1000u + tsim::kSampleOffsetNs / 1000u);
        TSIM_CHECK(sample.timestamp_us == expected_us);
    }
    TSIM_CHECK(firmware_status()->ticks == 20u);
}

TSIM_TEST(current_loop_tracks_a_step_on_a_locked_rotor) {
    tsim::IdealPlant plant(tsim::maxon_ec_i_40_70w_params());
    prepare_locked_plant(plant);
    firmware_set_current_reference_a(kCurrentReferenceA);

    std::vector<double> iq;
    iq.reserve(kCurrentLoopTicks);
    double worst_id = 0.0;
    double peak_iq = 0.0;
    for (std::uint32_t tick = 0; tick < kCurrentLoopTicks; ++tick) {
        sim_run_window(plant, tick);
        const firmware_status_t *status = firmware_status();
        iq.push_back(static_cast<double>(status->iq_a));
        worst_id = std::max(worst_id, std::fabs(static_cast<double>(status->id_a)));
        peak_iq = std::max(peak_iq, static_cast<double>(status->iq_a));
    }

    const double t10_s = crossing_time_s(iq, 0.1 * kCurrentReferenceA, tsim::kControlPeriodS);
    const double t90_s = crossing_time_s(iq, 0.9 * kCurrentReferenceA, tsim::kControlPeriodS);
    TSIM_CHECK(t10_s >= 0.0);
    TSIM_CHECK(t90_s >= 0.0);
    const double rise_us = (t90_s - t10_s) * 1.0e6;

    /* Delay free prediction from the design: one pole at Kp / L gives a 159 us
     * time constant, and a first order rise of 10 to 90 percent is 2.2 of them,
     * so 350 us. The measured value is allowed to be slower because the loop
     * has 75 us of pipeline delay that the prediction does not. */
    TSIM_CHECK(rise_us > 80.0);
    TSIM_CHECK(rise_us < 500.0);

    double steady_iq = 0.0;
    for (std::size_t i = 80; i < iq.size(); ++i) {
        steady_iq += iq[i];
    }
    steady_iq /= static_cast<double>(iq.size() - 80);
    TSIM_CHECK(std::fabs(steady_iq - kCurrentReferenceA) < 0.02);
    TSIM_CHECK(worst_id < 0.02);
    /* A first order approach does not cross its reference, so the peak is
     * allowed to sit just under it. Overshoot is what the upper bound is for. */
    TSIM_CHECK(peak_iq < 1.1 * kCurrentReferenceA);
    TSIM_CHECK(peak_iq >= 0.99 * kCurrentReferenceA);

    /* Locked rotor: the shaft stays where it was, so the measurement is of the
     * current loop and nothing else. */
    TSIM_CHECK(plant.mechanical().omega_rotor_rad_s == 0.0);
    TSIM_CHECK(plant.mechanical().theta_rotor_rad == 0.0);
}

TSIM_TEST(current_loop_recovers_after_the_reference_saturates) {
    /* A sagging bus is what saturates a current loop in practice, so the test
     * sags it: 8 V gives a 4.4 V voltage limit and a 20 A reference needs
     * 5.8 V across 0.291 ohm. The reference survives the firmware clamp
     * because the clamp is at stall current. */
    tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    params.bus_voltage_v = 8.0;
    tsim::IdealPlant plant(params);
    prepare_locked_plant(plant);
    firmware_set_current_reference_a(20.0f);

    double lowest_duty = 1.0;
    double highest_duty = 0.0;
    double largest_vq_v = 0.0;
    for (std::uint32_t tick = 0; tick < 20; ++tick) {
        sim_run_window(plant, tick);
        tsim::InverterCommand command{};
        sim_hal_take_inverter_command(&command);
        for (int phase = 0; phase < 3; ++phase) {
            const double duty = (phase == 0)   ? command.duty_a
                                : (phase == 1) ? command.duty_b
                                               : command.duty_c;
            lowest_duty = std::min(lowest_duty, duty);
            highest_duty = std::max(highest_duty, duty);
        }
        largest_vq_v =
            std::max(largest_vq_v, std::fabs(static_cast<double>(firmware_status()->vq_v)));
    }
    TSIM_CHECK(lowest_duty >= 0.0);
    TSIM_CHECK(highest_duty <= 1.0);
    const double voltage_limit_v = 0.95 * 0.57735027 * 8.0;
    TSIM_CHECK(largest_vq_v <= voltage_limit_v + 1.0e-3);
    TSIM_CHECK(largest_vq_v >= 0.9 * voltage_limit_v);

    /* Drop the reference and ask whether the integrator unwinds or sits there. */
    firmware_set_current_reference_a(1.0f);
    for (std::uint32_t tick = 20; tick < 100; ++tick) {
        sim_run_window(plant, tick);
    }
    const double recovered_iq = static_cast<double>(firmware_status()->iq_a);
    TSIM_CHECK(std::fabs(recovered_iq - 1.0) < 0.05);
}

TSIM_TEST(speed_loop_tracks_a_step_and_rejects_a_load) {
    tsim::IdealPlant plant(tsim::maxon_ec_i_40_70w_params());
    plant.reset(0.0);
    sim_hal_attach(plant);
    firmware_init();
    firmware_set_mode(FIRMWARE_MODE_SPEED);
    firmware_set_speed_reference_rad_s(kSpeedReferenceRadS);

    double peak_speed = 0.0;
    double lowest_speed = 1.0e9;
    double lowest_after_load = 1.0e9;
    double speed_before_load = 0.0;
    for (std::uint32_t tick = 0; tick < kSpeedLoopTicks; ++tick) {
        plant.set_external_torque_nm((tick >= kLoadStepTick) ? kLoadTorqueNm : 0.0);
        sim_run_window(plant, tick);
        const double speed = plant.mechanical().omega_rotor_rad_s;
        peak_speed = std::max(peak_speed, speed);
        /* Skip the first velocity window, during which the shaft is at rest by
         * definition and no torque has been commanded yet. After that a zero
         * speed would mean the loop had reversed the joint. */
        if (tick > HAL_VELOCITY_DIVIDER) {
            lowest_speed = std::min(lowest_speed, speed);
        }
        if (tick >= kLoadStepTick) {
            lowest_after_load = std::min(lowest_after_load, speed);
        }
        if (tick + 1 == kLoadStepTick) {
            speed_before_load = speed;
        }
    }

    const double final_speed = plant.mechanical().omega_rotor_rad_s;
    const double final_iq = static_cast<double>(firmware_status()->iq_a);

    TSIM_CHECK(std::fabs(speed_before_load - kSpeedReferenceRadS) < 0.02 * kSpeedReferenceRadS);
    TSIM_CHECK(std::fabs(final_speed - kSpeedReferenceRadS) < 0.01 * kSpeedReferenceRadS);
    /* Command filter and loop together: a two percent overshoot budget on a
     * step, and no reversal at any point. */
    TSIM_CHECK(peak_speed < 1.02 * kSpeedReferenceRadS);
    TSIM_CHECK(lowest_speed > 0.0);
    /* Disturbance rejection, measured as the dip a 50 mNm load step causes. */
    const double dip_rad_s = kSpeedReferenceRadS - lowest_after_load;
    TSIM_CHECK(dip_rad_s < 0.20 * kSpeedReferenceRadS);

    /* The load is 50 mNm and the torque constant is 46.1 mNm/A, so holding it
     * takes 1.08 A plus 0.04 A for viscous friction at 200 rad/s. A sign error
     * anywhere in the torque or q axis chain moves this to the wrong side. */
    TSIM_CHECK_NEAR(final_iq, 1.13, 0.1);
    TSIM_CHECK(plant.mechanical().theta_rotor_rad > 0.0);
    TSIM_CHECK(firmware_status()->fault_bits == 0u);
}

TSIM_TEST(feedforward_takes_the_d_axis_error_out_of_a_speed_transient) {
    /* Day 2 left the cross coupling and the back EMF to the PI and paid 0.071 A
     * of d axis current for it during this step. Both terms are now fed
     * forward from the measured current and the 2 kHz speed estimate, so what
     * is left is the error in that estimate, not the disturbance itself. */
    tsim::IdealPlant plant(tsim::maxon_ec_i_40_70w_params());
    plant.reset(0.0);
    sim_hal_attach(plant);
    firmware_init();
    firmware_set_mode(FIRMWARE_MODE_SPEED);
    firmware_set_speed_reference_rad_s(kSpeedReferenceRadS);

    double worst_id = 0.0;
    double worst_id_before_rotation = 0.0;
    for (std::uint32_t tick = 0; tick < kSpeedLoopTicks; ++tick) {
        plant.set_external_torque_nm((tick >= kLoadStepTick) ? kLoadTorqueNm : 0.0);
        sim_run_window(plant, tick);
        const double id = std::fabs(static_cast<double>(firmware_status()->id_a));
        if (tick > 2 * HAL_VELOCITY_DIVIDER) {
            worst_id = std::max(worst_id, id);
            worst_id_before_rotation = std::max(worst_id_before_rotation, id);
        }
    }
    TSIM_CHECK(worst_id < 0.035);
    TSIM_CHECK(worst_id_before_rotation < 0.035);
    /* The feedforward terms have to be doing something, not reading zero. */
    TSIM_CHECK(std::fabs(static_cast<double>(firmware_status()->vq_ff_v)) > 1.0);
    TSIM_CHECK(std::fabs(static_cast<double>(firmware_status()->vd_ff_v)) > 0.01);
    TSIM_CHECK(firmware_status()->dead_time_compensation);
}

TSIM_TEST(dead_time_compensation_removes_most_of_the_current_distortion) {
    /* Low speed on the switching plant: 30 rad/s is 33 Hz electrical, where the
     * back EMF is small enough that the 0.36 V dead time error is a large part
     * of the applied voltage. The measure is the ripple of the q axis current
     * about its own mean, taken over the last electrical cycle, because dead
     * time distortion lands on the sixth harmonic of the electrical frequency
     * and that harmonic is what shows up in q. The load is 90 mNm, so the
     * current is 1.95 A and the distortion is not buried under the resolution
     * of the measurement itself. */
    const float reference_rad_s = 30.0f;
    const std::uint32_t ticks = 4000;
    const std::uint32_t cycle_ticks = 600; /* one electrical cycle at 33 Hz */
    const std::uint32_t first = ticks - cycle_ticks;

    auto measure = [&](bool compensation) {
        tsim::SwitchingPlant plant(tsim::maxon_ec_i_40_70w_params(), realistic_inverter());
        plant.reset(0.0);
        sim_hal_attach(plant);
        sim_hal_set_board_dead_time_s(500.0e-9f);
        sim_hal_set_imperfections(false);
        firmware_init();
        firmware_set_dead_time_compensation(compensation);
        firmware_set_mode(FIRMWARE_MODE_SPEED);
        firmware_set_speed_reference_rad_s(reference_rad_s);

        std::vector<double> iq;
        for (std::uint32_t tick = 0; tick < ticks; ++tick) {
            plant.set_external_torque_nm(0.09);
            sim_run_window(plant, tick);
            if (tick >= first) {
                iq.push_back(static_cast<double>(firmware_status()->iq_a));
            }
        }
        double mean = 0.0;
        for (const double value : iq) {
            mean += value;
        }
        mean /= static_cast<double>(iq.size());
        double energy = 0.0;
        for (const double value : iq) {
            energy += (value - mean) * (value - mean);
        }
        return std::sqrt(energy / static_cast<double>(iq.size()));
    };

    const double off_ripple_a = measure(false);
    const double on_ripple_a = measure(true);
    TSIM_CHECK(off_ripple_a > 0.01);
    TSIM_CHECK(on_ripple_a < 0.3 * off_ripple_a);
}
