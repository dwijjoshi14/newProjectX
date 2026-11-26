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

TSIM_TEST(position_mode_holds_where_it_was_entered) {
    /* A mode change must not command a move. The reference is seeded to the
     * present position, so a joint switched into position mode with no further
     * command stays exactly where the handle was. */
    /* No hand spring here on purpose. At 0.3 rad a 0.45 N m/rad hand spring pulls
     * with 135 mN m, which is 2.9 A of q axis current against a speed loop
     * clamped at 2.74 A: the joint would sag to where the spring torque equals
     * the clamp. That is a real limit and it is tested below, but it is not what
     * this test is asking. */
    tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    params.mechanics = tsim::mechanics_params_compliant_load();
    tsim::IdealPlant plant(params);
    plant.reset(0.3);
    sim_hal_attach(plant);
    firmware_init();
    firmware_set_mode(FIRMWARE_MODE_POSITION);

    const double start_rad = plant.mechanical().theta_rotor_rad;
    for (std::uint32_t tick = 0; tick < 1000; ++tick) { /* 50 ms */
        sim_run_window(plant, tick);
    }
    TSIM_CHECK(std::fabs(plant.mechanical().theta_rotor_rad - start_rad) < 2.0e-3);
    TSIM_CHECK(std::fabs(firmware_status()->position_error_rad) < 2.0e-3);
    TSIM_CHECK_NEAR(firmware_status()->position_ref_rad, start_rad, 1.0e-3);
}

TSIM_TEST(position_loop_tracks_a_step_through_the_speed_cascade) {
    /* 0.1 rad on the joint with its hand attached. The design puts the position
     * crossover at 25 Hz, so a first order rise to 90 percent is 15 ms and the
     * speed loop underneath and the 1 kHz outer rate are allowed to make it
     * slower than that. The steady state error is the interesting number: a P
     * position loop with an integral speed loop has none, and Coulomb friction
     * is the thing that would show up as one if the cascade were wrong. */
    tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    params.mechanics = tsim::mechanics_params_hand();
    tsim::IdealPlant plant(params);
    plant.reset(0.0);
    sim_hal_attach(plant);
    firmware_init();
    firmware_set_mode(FIRMWARE_MODE_POSITION);
    firmware_set_position_reference_rad(0.1f);

    const std::uint32_t ticks = 16000; /* 800 ms, long enough to leave the band */
    std::vector<double> position;
    position.reserve(ticks);
    double peak_rad = 0.0;
    for (std::uint32_t tick = 0; tick < ticks; ++tick) {
        sim_run_window(plant, tick);
        const double theta_rad = plant.mechanical().theta_rotor_rad;
        position.push_back(theta_rad);
        peak_rad = std::max(peak_rad, theta_rad);
    }
    const double t10_s = crossing_time_s(position, 0.01, tsim::kControlPeriodS);
    const double t90_s = crossing_time_s(position, 0.09, tsim::kControlPeriodS);
    TSIM_CHECK(t10_s >= 0.0);
    TSIM_CHECK(t90_s >= 0.0);
    const double rise_ms = (t90_s - t10_s) * 1.0e3;
    TSIM_CHECK(rise_ms > 5.0);
    /* Half a second, which is not what a 80 per second gain would give on a
     * linear plant. The joint spends it crossing its own friction band: the
     * shaft has 8 mN m of stiction and the hand is 0.45 N m/rad, so the spring
     * has to be wound a degree past the reference before the joint breaks free
     * and a degree short before it holds. The band is the joint, not the tuning. */
    TSIM_CHECK(rise_ms < 700.0);

    /* 15.9 percent overshoot, and the bound is 20. That is more than the speed
     * loop's own 1.6 percent, and the reason is the compliant coupling: the two
     * mass resonance sits at 100 Hz, which is where the speed loop crosses over,
     * so the outer loop is asking the inner one to work at its least stable
     * frequency. docs/results.md runs the same step with a rigid coupling and the
     * overshoot halves, which is the same effect seen from the other side. */
    TSIM_CHECK(peak_rad < 0.12);

    /* Nine milliradians short, and the bound is 0.45 N m/rad times the error
     * against 8 mN m of stiction: 17.8 mrad is where the hand spring stops being
     * able to push the joint out of the friction band, so a P position loop with
     * a stick slip shaft can legitimately stop anywhere inside it. Getting
     * tighter needs friction compensation or an integral in the position loop,
     * and neither is here. The measured 9.3 mrad is inside the band, not a gain
     * error, and this test would fail if the band grew. */
    const double stiction_band_rad = 8.0e-3 / 0.45;
    const double final_rad = plant.mechanical().theta_rotor_rad;
    /* Anywhere the spring force is inside the friction band is a place this
     * joint can stop, so the check is two bands wide and not a point. Inside
     * that band the loop hunts, because the velocity estimate is quantised to
     * 0.77 rad/s and the speed PI is asked to work below one count. The hunt is
     * the subject of docs/bringup-log.md on day 4, not something this test can
     * hide. */
    TSIM_CHECK_NEAR(final_rad, 0.1, 2.0 * stiction_band_rad);
    TSIM_CHECK(std::fabs(firmware_status()->position_error_rad) < 2.0 * stiction_band_rad);
    TSIM_CHECK(firmware_status()->fault_bits == 0u);
}

TSIM_TEST(impedance_renders_the_commanded_stiffness_against_a_wall) {
    /* A spring of 1 N m/rad pushing to 0.15 rad against a wall at 0.05 rad with
     * 1.8 N m/rad of its own. The static balance is one line:
     * 1.0 (0.15 - theta) = 1.8 (theta - 0.05), so theta is 0.0857 rad and the
     * torque is 64 mN m. Friction of 8 mN m widens that into a band, which is
     * why the check is a band and not a point. */
    tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    params.mechanics = tsim::mechanics_params_wall(0.05);
    tsim::IdealPlant plant(params);
    plant.reset(0.0);
    sim_hal_attach(plant);
    firmware_init();
    firmware_set_impedance(1.0f, 0.05f);
    firmware_set_mode(FIRMWARE_MODE_IMPEDANCE);
    firmware_set_position_reference_rad(0.15f);

    for (std::uint32_t tick = 0; tick < 4000; ++tick) { /* 200 ms */
        sim_run_window(plant, tick);
    }

    /* And this is the finding, not the arithmetic I wrote down first: the
     * rendered spring is in series with everything between it and the wall, so
     * the handle feels 1 / (1/1.0 + 1/250 + 1/1.8) = 0.64 N m/rad and not the
     * 1.0 that was commanded. Commanding 0.1 rad of total compression gives
     * 64 mN m at the wall instead of 100. The transmission barely matters now
     * that it is a shaft; the wall's own 1.8 N m/rad is what costs the most,
     * and a stiffer wall would take the rendered stiffness closer to the command.
     * Rendering a stiffness at the handle exactly needs a torque sensor to close
     * the loop on the contact, which this drive does not have. */
    const double theta_rad = plant.mechanical().theta_rotor_rad;
    TSIM_CHECK(theta_rad > 0.080);
    TSIM_CHECK(theta_rad < 0.092);
    /* The shaft's own 8 mN m of stiction widens both numbers by that much, so
     * the checks are the band the joint can legitimately stop in. */
    TSIM_CHECK_NEAR(firmware_status()->torque_ref_nm, 0.0641, 0.008);
    TSIM_CHECK_NEAR(plant.mechanical().torque_wall_nm, 0.0641, 0.006);
    /* The joint is holding against the wall, not oscillating through it. */
    TSIM_CHECK(std::fabs(plant.mechanical().omega_rotor_rad_s) < 1.0);
    TSIM_CHECK(firmware_status()->stiffness_nm_per_rad == 1.0f);
}

TSIM_TEST(impedance_holds_a_large_angle_without_losing_counts) {
    /* 300 rad is forty seven turns, and a float angle there has a resolution of
     * 30 microrad. The error is computed in counts and converted afterwards, so
     * a 0.005 rad offset at this angle still resolves. */
    tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    params.mechanics = tsim::mechanics_params_compliant_load();
    tsim::IdealPlant plant(params);
    plant.reset(300.0);
    sim_hal_attach(plant);
    firmware_init();
    firmware_set_impedance(2.0f, 0.05f);
    firmware_set_mode(FIRMWARE_MODE_IMPEDANCE);
    firmware_set_position_reference_rad(300.005f);

    for (std::uint32_t tick = 0; tick < 4000; ++tick) {
        sim_run_window(plant, tick);
    }
    TSIM_CHECK_NEAR(plant.mechanical().theta_rotor_rad, 300.005, 1.0e-3);
    /* The joint stops inside the stiction band, not at zero error: 8 mN m of
     * Coulomb friction against 2 N m/rad is 4 mrad either side, which is eleven
     * counts. That band is the resolution of a rendered position, and the last
     * test to mention it. */
    const double one_count_rad = tsim::kTwoPi / 16384.0;
    TSIM_CHECK(std::fabs(firmware_status()->position_error_rad) < 0.0055);
    TSIM_CHECK(std::fabs(firmware_status()->position_error_rad) > 0.5 * one_count_rad);
    TSIM_CHECK(plant.mechanical().theta_rotor_rad > 300.0);
}

namespace {

/* One run of an impedance render whose reference is the joint's own angle from
 * five milliseconds ago. That is the delayed-coupling mechanism with the link
 * and the second joint taken out: the reference arrives late, the spring is
 * stretched by a position the drive did not see, and the drive does work it
 * never stored. If the observer cannot see this it cannot see a bilateral pair.
 */
struct DelayedReferenceRun {
    double excess_min_j = 0.0;
    double peak_early_rad_s = 0.0;
    double peak_late_rad_s = 0.0;
    double damping_max_nm_s_per_rad = 0.0;
};

DelayedReferenceRun delayed_reference_run(bool passivity_control) {
    tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    params.mechanics = tsim::mechanics_params_rigid();
    tsim::SwitchingPlant plant(params, tsim::inverter_params_realistic());
    plant.reset(0.0);
    sim_hal_attach(plant);
    sim_hal_set_board_dead_time_s(5.0e-7f);
    sim_hal_set_imperfections(false);
    sim_hal_latch_sample(plant.sample_electrical(), 0u);

    firmware_init();
    firmware_set_passivity_control(passivity_control);
    firmware_set_impedance(8.0f, 0.0f);
    firmware_set_mode(FIRMWARE_MODE_IMPEDANCE);

    constexpr std::uint64_t kTicks = 4000; /* 200 ms */
    constexpr std::uint64_t kDelay = 100;  /* 5 ms */
    std::vector<double> history;
    history.reserve(kTicks + 1u);
    DelayedReferenceRun run;
    for (std::uint64_t tick = 0; tick < kTicks; ++tick) {
        history.push_back(plant.mechanical().theta_rotor_rad);
        /* A fixed offset for the first 10 ms to put some energy in, then the
         * delayed self reference. */
        const double reference =
            (tick < 200u) ? 0.01 : history[static_cast<std::size_t>(tick - kDelay)];
        firmware_set_position_reference_rad(static_cast<float>(reference));
        sim_run_window(plant, tick);

        const double excess_j = firmware_status()->passivity_excess_j;
        run.excess_min_j = std::min(run.excess_min_j, excess_j);
        const double speed_rad_s = std::fabs(plant.mechanical().omega_rotor_rad_s);
        run.damping_max_nm_s_per_rad =
            std::max(run.damping_max_nm_s_per_rad,
                     static_cast<double>(firmware_status()->passivity_damping_nm_s_per_rad));
        if (tick < kTicks / 4u) {
            run.peak_early_rad_s = std::max(run.peak_early_rad_s, speed_rad_s);
        }
        if (tick >= 3u * kTicks / 4u) {
            run.peak_late_rad_s = std::max(run.peak_late_rad_s, speed_rad_s);
        }
    }
    return run;
}

} // namespace

TSIM_TEST(passivity_observer_catches_a_reference_that_arrives_late) {
    /* Without the controller the observer has to go clearly negative: the render
     * is returning energy, and that number is the evidence. With the controller
     * the excess has to stay at the noise floor, because the damping it adds is
     * sized to dissipate exactly what the observer sees. */
    const DelayedReferenceRun open_loop = delayed_reference_run(false);
    const DelayedReferenceRun controlled = delayed_reference_run(true);

    TSIM_CHECK(open_loop.excess_min_j < -0.1);
    /* What is not asserted here is that the controlled run generates less. In this
     * case the reference is the joint's own past angle, so the controller changes
     * the trajectory and the loop then generates a different amount: measured, the
     * controlled run can generate more. A one port observer cannot stabilise a loop
     * whose reference is one of its own states, and a test that pretended otherwise
     * would be the test lying about the method. What the controller buys is
     * measured in the bilateral study, where the reference is another joint. */
    TSIM_CHECK(controlled.damping_max_nm_s_per_rad > 0.0);
    /* The controller has to have done something: with it off the damping is
     * exactly zero, and with it on it is not. */
    TSIM_CHECK(open_loop.damping_max_nm_s_per_rad == 0.0);
    TSIM_CHECK(controlled.damping_max_nm_s_per_rad > 0.01);
    /* What this test does NOT assert is that the joint stops. It does not: a
     * one port observer with a reference that is free to move sees only the
     * energy its own render returns, and a loop that feeds its own delayed
     * position back in can keep that render supplied. Measured, the joint still
     * reaches the current limit in both runs. The controller bounds what the
     * render gives back, which is the safety claim, and it is not a stabiliser
     * for an arbitrary delayed coupling. docs/results.md says so. */
    TSIM_CHECK(controlled.peak_late_rad_s > 0.0);
}

TSIM_TEST(passivity_observer_stays_quiet_on_a_passive_spring) {
    /* A spring and a damper against a fixed reference, pushed by a hand and
     * released, is passive by construction. The reference is set once, before
     * anything moves, so nothing in this run is a reference motion the observer
     * could mistake for an injection: the energy goes in and comes out through
     * the port, which is the case the observer has to leave alone. If it did not,
     * the drive would damp every contact it renders and the stiffness day 4
     * measured would have been the controller's, not the spring's. */
    tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    params.mechanics = tsim::mechanics_params_rigid();
    tsim::SwitchingPlant plant(params, tsim::inverter_params_realistic());
    plant.reset(0.0);
    sim_hal_attach(plant);
    sim_hal_set_board_dead_time_s(5.0e-7f);
    sim_hal_set_imperfections(false);
    sim_hal_latch_sample(plant.sample_electrical(), 0u);

    firmware_init();
    firmware_set_passivity_control(true);
    firmware_set_impedance(4.0f, 0.02f);
    firmware_set_mode(FIRMWARE_MODE_IMPEDANCE);
    firmware_set_position_reference_rad(0.0f);

    double excess_min_j = 0.0;
    double largest_damping_nm_s_per_rad = 0.0;
    double largest_angle_rad = 0.0;
    for (std::uint64_t tick = 0; tick < 4000u; ++tick) {
        /* 20 mN m for 20 ms, then nothing: a push into the spring and a release. */
        plant.set_external_torque_nm((tick >= 400u && tick < 800u) ? 0.02 : 0.0);
        sim_run_window(plant, tick);
        const firmware_status_t *status = firmware_status();
        excess_min_j = std::min(excess_min_j, static_cast<double>(status->passivity_excess_j));
        largest_damping_nm_s_per_rad =
            std::max(largest_damping_nm_s_per_rad,
                     static_cast<double>(status->passivity_damping_nm_s_per_rad));
        largest_angle_rad =
            std::max(largest_angle_rad, std::fabs(plant.mechanical().theta_rotor_rad));
    }
    /* The floor is the encoder count, not the physics: the stored energy is built
     * from an error quantised to 0.38 mrad, and that wobbles the observer by a
     * few microjoules. A render that leaks a millijoule is a different thing. */
    TSIM_CHECK(excess_min_j > -5.0e-4);
    TSIM_CHECK(largest_damping_nm_s_per_rad < 2.0);
    /* A spring of 4 N m/rad pushed with 20 mN m for 20 ms oscillates, and the
     * point is that it is still there to oscillate: the controller did not eat
     * the contact. */
    TSIM_CHECK(largest_angle_rad > 0.02);
    TSIM_CHECK(plant.mechanical().theta_rotor_rad > -0.01);
}
