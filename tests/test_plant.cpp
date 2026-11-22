#include <cmath>

#include "tsim/ideal_plant.h"
#include "tsim/plant.h"
#include "tsim/plant_params.h"
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

} // namespace

TSIM_TEST(rl_step_matches_analytic) {
    tsim::IdealPlant plant(tsim::maxon_ec_i_40_70w_params());
    plant.reset(0.0);

    /* Alpha aligned voltage at zero electrical angle is pure d axis current, so
     * no torque is produced and the rotor stays put. */
    const double voltage_alpha_v = 3.6;
    plant.latch_inverter(duty_from_alpha_beta(voltage_alpha_v, 0.0, plant.params().bus_voltage_v));

    const std::uint32_t steps = 2000; /* 2 ms */
    run_steps(plant, steps);

    const double time_s = static_cast<double>(steps) * tsim::kPlantStepS;
    const double tau_s = plant.params().inductance_phase_h / plant.params().resistance_phase_ohm;
    const double current_inf_a = voltage_alpha_v / plant.params().resistance_phase_ohm;
    const double expected_a = current_inf_a * (1.0 - std::exp(-time_s / tau_s));

    TSIM_CHECK_NEAR(plant.sample_electrical().ia_a, expected_a, 5.0e-3);
    TSIM_CHECK(std::fabs(plant.mechanical().torque_motor_nm) < 1.0e-9);
    TSIM_CHECK(plant.mechanical().omega_rotor_rad_s == 0.0);
}

TSIM_TEST(dc_steady_state_matches_ohms_law) {
    tsim::IdealPlant plant(tsim::maxon_ec_i_40_70w_params());
    plant.reset(0.0);

    const double voltage_alpha_v = 3.6;
    plant.latch_inverter(duty_from_alpha_beta(voltage_alpha_v, 0.0, plant.params().bus_voltage_v));
    run_steps(plant, 30000); /* 30 ms, about 34 electrical time constants */

    const double expected_a = voltage_alpha_v / plant.params().resistance_phase_ohm;
    TSIM_CHECK_NEAR(plant.sample_electrical().ia_a, expected_a, 2.0e-3);

    const tsim::Telemetry telemetry = plant.telemetry();
    TSIM_CHECK(telemetry.p_electrical_w > 0.0);
    TSIM_CHECK(telemetry.p_mechanical_w == 0.0);
    TSIM_CHECK_NEAR(telemetry.p_electrical_w, telemetry.p_copper_w, 5.0e-3);
}

TSIM_TEST(open_bridge_clears_current) {
    tsim::IdealPlant plant(tsim::maxon_ec_i_40_70w_params());
    plant.reset(0.0);
    plant.latch_inverter(duty_from_alpha_beta(3.6, 0.0, plant.params().bus_voltage_v));
    run_steps(plant, 1000);
    TSIM_CHECK(std::fabs(plant.sample_electrical().ia_a) > 0.1);

    tsim::InverterCommand open{};
    open.bridge_enabled = false;
    plant.latch_inverter(open);
    plant.step_one();

    TSIM_CHECK(plant.sample_electrical().ia_a == 0.0);
    TSIM_CHECK(plant.sample_electrical().ib_a == 0.0);
    TSIM_CHECK(plant.sample_electrical().ic_a == 0.0);
}

TSIM_TEST(coast_down_is_viscous_only) {
    tsim::IdealPlant plant(tsim::maxon_ec_i_40_70w_params());
    plant.reset(0.0);

    /* Spin up with an external torque and open windings, so only inertia and
     * viscous friction are in play. */
    tsim::InverterCommand open{};
    open.bridge_enabled = false;
    plant.latch_inverter(open);
    plant.set_external_torque_nm(-0.02);
    run_steps(plant, 10000); /* 10 ms */
    plant.set_external_torque_nm(0.0);

    const double omega_zero_rad_s = plant.mechanical().omega_rotor_rad_s;
    TSIM_CHECK(omega_zero_rad_s > 10.0);

    const tsim::EnergyAccount energy_before = plant.energy();
    const std::uint32_t steps = 50000; /* 50 ms */
    run_steps(plant, steps);

    const double time_s = static_cast<double>(steps) * tsim::kPlantStepS;
    const double tau_s =
        plant.params().inertia_kg_m2 / plant.params().viscous_friction_nm_s_per_rad;
    const double expected_rad_s = omega_zero_rad_s * std::exp(-time_s / tau_s);
    TSIM_CHECK_NEAR(plant.mechanical().omega_rotor_rad_s, expected_rad_s, 2.0e-3);

    const tsim::EnergyAccount energy_after = plant.energy();
    const double kinetic_before_j =
        0.5 * plant.params().inertia_kg_m2 * omega_zero_rad_s * omega_zero_rad_s;
    const double kinetic_after_j = 0.5 * plant.params().inertia_kg_m2 *
                                   plant.mechanical().omega_rotor_rad_s *
                                   plant.mechanical().omega_rotor_rad_s;
    const double lost_kinetic_j = kinetic_before_j - kinetic_after_j;
    const double dissipated_j = energy_after.viscous_j - energy_before.viscous_j;

    TSIM_CHECK_NEAR(dissipated_j, lost_kinetic_j, 2.0e-3);
    TSIM_CHECK(energy_after.copper_j == energy_before.copper_j);
}

TSIM_TEST(plant_time_is_exact) {
    tsim::IdealPlant plant(tsim::maxon_ec_i_40_70w_params());
    plant.reset(0.0);
    run_steps(plant, static_cast<std::uint32_t>(tsim::kStepsPerControl));
    TSIM_CHECK(plant.step_count() == tsim::kStepsPerControl);
    TSIM_CHECK_NEAR(plant.time_s(), tsim::kControlPeriodS, 1.0e-12);
}
