/* Tests for the switching inverter. Written before the plant implementation
 * ran: each one asserts a property of the physics, not a number the code
 * happened to produce.
 *
 * The first test is the one that matters most. Two implementations of the same
 * electrical equations now exist, the average model and the switching model,
 * and a silent divergence between them would make every day 1 and day 2 result
 * unreproducible without anyone noticing. The reducing case is checked here. */

#include <cmath>

#include "tsim/ideal_plant.h"
#include "tsim/plant_params.h"
#include "tsim/switching_plant.h"
#include "tsim_test.h"

namespace {

/* Duty vector for a requested alpha beta voltage, phase and inverter frames.
 * Amplitude invariant inverse Clarke, common mode at 0.5. */
tsim::InverterCommand duty_from_alpha_beta(double voltage_alpha_v, double voltage_beta_v,
                                           double bus_voltage_v) {
    const double root_three_over_two = 0.8660254037844386;
    const double v_a = voltage_alpha_v;
    const double v_b = -0.5 * voltage_alpha_v + root_three_over_two * voltage_beta_v;
    const double v_c = -0.5 * voltage_alpha_v - root_three_over_two * voltage_beta_v;
    tsim::InverterCommand command{};
    command.duty_a = static_cast<float>(0.5 + v_a / bus_voltage_v);
    command.duty_b = static_cast<float>(0.5 + v_b / bus_voltage_v);
    command.duty_c = static_cast<float>(0.5 + v_c / bus_voltage_v);
    command.bridge_enabled = true;
    return command;
}

void run_steps(tsim::IPlant &plant, std::uint32_t steps) {
    for (std::uint32_t i = 0; i < steps; ++i) {
        plant.step_one();
    }
}

tsim::InverterParams dead_time_only(double dead_time_s) {
    tsim::InverterParams inverter = tsim::inverter_params_ideal();
    inverter.dead_time_s = dead_time_s;
    inverter.dead_time_linear_a = 0.5;
    return inverter;
}

/* Duty that holds a positive phase A current on a locked rotor, and the same
 * vector mirrored for a negative one. */
tsim::InverterCommand positive_duty() {
    tsim::InverterCommand command{};
    command.duty_a = 0.6f;
    command.duty_b = 0.4f;
    command.duty_c = 0.5f;
    command.bridge_enabled = true;
    return command;
}

} // namespace

TSIM_TEST(the_switching_plant_reduces_to_the_average_model) {
    /* Same motor, no dead time, no device resistance, no loss, stiff bus. The
     * two plants must then agree, sample by sample, over a rotating voltage
     * vector that exercises all six sectors. */
    const tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    tsim::IdealPlant average(params);
    tsim::SwitchingPlant switching(params, tsim::inverter_params_ideal());
    average.reset(0.0);
    switching.reset(0.0);

    const double window_s = static_cast<double>(tsim::kStepsPerControl) * tsim::kPlantStepS;
    double worst = 0.0;
    for (std::uint32_t window = 0; window < 20; ++window) {
        const double t_s = static_cast<double>(window) * window_s;
        const double angle = 2.0 * tsim::kTwoPi * 37.0 * t_s;
        const tsim::InverterCommand command = duty_from_alpha_beta(
            4.0 * std::cos(angle), 4.0 * std::sin(angle), params.bus_voltage_v);
        average.latch_inverter(command);
        switching.latch_inverter(command);
        for (std::uint32_t i = 0; i < tsim::kStepsPerControl; ++i) {
            average.step_one();
            switching.step_one();
        }
        const tsim::ElectricalReadout a = average.sample_electrical();
        const tsim::ElectricalReadout b = switching.sample_electrical();
        worst = std::max(worst, std::fabs(a.ia_a - b.ia_a));
        worst = std::max(worst, std::fabs(a.ib_a - b.ib_a));
        worst = std::max(worst, std::fabs(a.ic_a - b.ic_a));
    }

    /* Not exact equality, and the reason is arithmetic rather than physics:
     * the average plant adds the three float duty ratios in float and divides
     * the sum by 3.0, while the switching plant removes the common mode from
     * three double voltages. That is a 1e-8 relative rounding difference in
     * the neutral offset, which lands here as a few hundred nanoamps on a ten
     * amp current. Anything larger would be a real divergence. */
    TSIM_CHECK(worst < 1.0e-5);
}

TSIM_TEST(dead_time_takes_a_volt_second_from_the_leg_carrying_positive_current) {
    const tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    tsim::SwitchingPlant plant(params, dead_time_only(500.0e-9));
    plant.reset(0.0);
    plant.set_rotor_locked(true);
    plant.latch_inverter(positive_duty());

    /* 200 steps is 200 us, about a quarter of an electrical time constant, so
     * the current is well above the 0.5 A where the sign is unambiguous. */
    run_steps(plant, 200);
    const tsim::ElectricalReadout positive = plant.sample_electrical();
    TSIM_CHECK(positive.ia_a > 0.5);

    /* The leg loses bus * dead_time / period of average voltage, 0.36 V. At
     * 60 percent against a 50 percent mean the differential is 3.6 V, so the
     * winding sees 3.24 V. The other two legs carry the opposite error and the
     * mean of the three is close to zero, so almost all of it survives. */
    const double design_v = 36.0 * (0.6 - 0.5) - 36.0 * (500.0e-9 / 50.0e-6);
    TSIM_CHECK_NEAR(positive.va_v, design_v, 0.01);

    tsim::SwitchingPlant mirrored(params, dead_time_only(500.0e-9));
    mirrored.reset(0.0);
    mirrored.set_rotor_locked(true);
    tsim::InverterCommand command = positive_duty();
    command.duty_a = 0.4f;
    command.duty_b = 0.6f;
    mirrored.latch_inverter(command);
    run_steps(mirrored, 200);
    const tsim::ElectricalReadout negative = mirrored.sample_electrical();
    TSIM_CHECK(negative.ia_a < -0.5);
    /* Negative current: the dead time error is now a gain, not a loss. */
    TSIM_CHECK_NEAR(negative.va_v, -3.6 + 36.0 * (500.0e-9 / 50.0e-6), 0.01);
}

TSIM_TEST(device_resistance_is_in_series_with_the_winding) {
    const tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    tsim::InverterParams inverter = tsim::inverter_params_ideal();
    inverter.on_resistance_ohm = 3.0e-3;
    tsim::SwitchingPlant plant(params, inverter);
    plant.reset(0.0);
    plant.set_rotor_locked(true);
    plant.latch_inverter(positive_duty());
    run_steps(plant, 30000); /* 30 ms, about 34 electrical time constants */

    /* One device conducts in each leg, so the phase resistance is R plus Ron.
     * The neutral shifts but the sum of the three current errors is zero. */
    const double design_a = 3.6 / (params.resistance_phase_ohm + 3.0e-3);
    TSIM_CHECK_NEAR(plant.sample_electrical().ia_a, design_a, 0.01);
}

TSIM_TEST(the_link_capacitor_moves_at_the_switching_frequency) {
    const tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    tsim::SwitchingPlant plant(params, tsim::inverter_params_realistic());
    plant.reset(0.0);
    plant.set_rotor_locked(true);
    tsim::InverterCommand command{};
    command.duty_a = 0.52f;
    command.duty_b = 0.48f;
    command.duty_c = 0.5f;
    command.bridge_enabled = true;
    plant.latch_inverter(command);
    run_steps(plant, 2000); /* 2 ms, past the 10 us source time constant */

    double lowest_v = 1.0e9;
    double highest_v = 0.0;
    for (std::uint32_t i = 0; i < tsim::kStepsPerControl; ++i) {
        plant.step_one();
        const double bus_v = plant.sample_electrical().bus_v;
        lowest_v = std::min(lowest_v, bus_v);
        highest_v = std::max(highest_v, bus_v);
    }
    /* A 100 uF link loaded at a couple of amps for a few microseconds per
     * period has to move, and it cannot move by more than the source
     * resistance would allow at DC. Both bounds are stated, because a ripple
     * of exactly zero would mean the capacitor is not in the model and a
     * ripple of volts would mean the model is unstable. */
    TSIM_CHECK(highest_v - lowest_v > 0.01);
    TSIM_CHECK(highest_v - lowest_v < 2.0);
    TSIM_CHECK(lowest_v > 30.0);
}

TSIM_TEST(switching_energy_is_counted_and_close_to_the_link_energy) {
    const tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    const tsim::InverterParams inverter = tsim::inverter_params_realistic();
    tsim::SwitchingPlant plant(params, inverter);
    plant.reset(0.0);
    plant.set_rotor_locked(true);
    plant.latch_inverter(positive_duty());
    const std::uint32_t windows = 200;
    run_steps(plant, windows * tsim::kStepsPerControl);

    /* The transition energy is the overlap triangle of a commutation, half of
     * bus times current times the crossover, and a leg does four device events
     * per period. So per leg per period it is 2 * bus * current * crossover,
     * and the check below is that number against the sum of what the three legs
     * carried. */
    const tsim::EnergyAccount energy = plant.energy();
    const tsim::ElectricalReadout settled_before = plant.sample_electrical();
    const double sum_abs_before_a = std::fabs(settled_before.ia_a) +
                                    std::fabs(settled_before.ib_a) + std::fabs(settled_before.ic_a);
    const double design_j = static_cast<double>(windows) * 2.0 * plant.params().bus_voltage_v *
                            sum_abs_before_a * inverter.switching_crossover_s;
    TSIM_CHECK_NEAR(energy.switching_j, design_j, 0.02);

    /* Every joule from the link has to land somewhere, and the locked rotor
     * stores no mechanical energy. The two losses that are not accounted for
     * by copper and by the transitions are the device drop and the dead time
     * volt seconds, both estimated from the steady state currents with the
     * same parameters the plant was given. A closing balance to 5 percent is
     * the check that the inverter is not silently creating or eating energy. */
    const tsim::ElectricalReadout settled = plant.sample_electrical();
    const double elapsed_s = static_cast<double>(windows) * tsim::kControlPeriodS;
    const double sum_abs_a =
        std::fabs(settled.ia_a) + std::fabs(settled.ib_a) + std::fabs(settled.ic_a);
    const double sum_square_a =
        settled.ia_a * settled.ia_a + settled.ib_a * settled.ib_a + settled.ic_a * settled.ic_a;
    const double dead_time_v =
        inverter.dead_time_s / tsim::kControlPeriodS * plant.params().bus_voltage_v;
    const double conduction_j =
        (inverter.on_resistance_ohm * sum_square_a + dead_time_v * sum_abs_a) * elapsed_s;
    const double predicted_j = energy.copper_j + energy.switching_j + conduction_j;
    TSIM_CHECK_NEAR(energy.electrical_j, predicted_j, 0.05);
    TSIM_CHECK(energy.mechanical_j == 0.0);
    /* The dead time loss is not a rounding correction on this machine: five
     * hundred nanoseconds of 36 V at twelve amps is watts, and it appears here
     * rather than only in the distortion. */
    TSIM_CHECK(conduction_j > 0.02 * energy.copper_j);
}

TSIM_TEST(resolved_switching_produces_winding_ripple) {
    const tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    tsim::SwitchingPlant plant(params, tsim::inverter_params_realistic());
    plant.reset(0.0);
    plant.set_rotor_locked(true);
    tsim::InverterCommand command{};
    command.duty_a = 0.6f;
    command.duty_b = 0.4f;
    command.duty_c = 0.5f;
    command.bridge_enabled = true;
    plant.latch_inverter(command);
    run_steps(plant, 2000);

    double lowest_a = 1.0e9;
    double highest_a = -1.0e9;
    for (std::uint32_t i = 0; i < tsim::kStepsPerControl; ++i) {
        plant.step_one();
        const double ia = plant.sample_electrical().ia_a;
        lowest_a = std::min(lowest_a, ia);
        highest_a = std::max(highest_a, ia);
    }
    /* The average model has no ripple at all, so any nonzero peak to peak here
     * is the switching being resolved rather than averaged. The bound is the
     * single leg triangle bound V D (1-D) / (L f) = 1.76 A: the winding sees
     * the differential of three legs, which cannot exceed that. */
    const double ripple_a = highest_a - lowest_a;
    TSIM_CHECK(ripple_a > 0.05);
    TSIM_CHECK(ripple_a < 1.76);
}

TSIM_TEST(average_duty_holds_no_winding_ripple) {
    /* All three legs at 50 percent switch together, so the differential
     * voltage stays zero and the winding current does not ripple at all. The
     * 1.76 A figure in docs/parameters.md is the single leg bound against the
     * bus midpoint, not what the winding carries when the three duties match,
     * and this test is what says so. */
    const tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    tsim::SwitchingPlant plant(params, tsim::inverter_params_realistic());
    plant.reset(0.0);
    plant.set_rotor_locked(true);
    tsim::InverterCommand command{};
    command.duty_a = 0.5f;
    command.duty_b = 0.5f;
    command.duty_c = 0.5f;
    command.bridge_enabled = true;
    plant.latch_inverter(command);
    run_steps(plant, 500);

    double lowest_a = 1.0e9;
    double highest_a = -1.0e9;
    for (std::uint32_t i = 0; i < tsim::kStepsPerControl; ++i) {
        plant.step_one();
        const double ia = plant.sample_electrical().ia_a;
        lowest_a = std::min(lowest_a, ia);
        highest_a = std::max(highest_a, ia);
    }
    TSIM_CHECK(highest_a - lowest_a < 1.0e-12);
}
