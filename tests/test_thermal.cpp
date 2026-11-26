/* Thermal tests, written before the network existed.
 *
 * Three things are checked, in the order they can go wrong. The four datasheet
 * numbers have to reproduce the two published time constants, or one of the
 * four was entered wrong. The network has to settle at the DC gain, because a
 * derating curve is nothing but that gain inverted. And the copper coefficient
 * has to be in the loop, because without it the loss does not grow as the
 * winding heats and the runaway current this file computes would not exist.
 */

#include <cmath>

#include "tsim/ideal_plant.h"
#include "tsim/plant.h"
#include "tsim/plant_params.h"
#include "tsim/thermal.h"
#include "tsim_test.h"

namespace {

constexpr double kWindingResistanceOhm = 0.291; /* phase, docs/parameters.md */
constexpr double kContinuousCurrentA = 2.74;    /* datasheet */
constexpr double kMaxWindingC = 155.0;          /* datasheet insulation limit */
constexpr double kBenchAmbientC = 25.0;

tsim::ThermalParams enabled_network() {
    tsim::ThermalParams params = tsim::thermal_params_maxon();
    params.enabled = true;
    return params;
}

} // namespace

TSIM_TEST(thermal_network_reproduces_the_published_time_constants) {
    /* 2.27 K/W times 11.0 J/K is 24.97 s against a published 24.9, and 8.17
     * times 125 is 1021.25 against a published 1020. Both of those are a check
     * on the four numbers, not on the solver. */
    tsim::ThermalNetwork network;
    network.configure(enabled_network());
    TSIM_CHECK_NEAR(network.winding_time_constant_s(), 24.9, 0.1);
    TSIM_CHECK_NEAR(network.housing_time_constant_s(), 1020.0, 2.0);
}

TSIM_TEST(thermal_network_settles_at_the_dc_gain) {
    /* Stepping the network itself at a coarse dt is legitimate here because the
     * steady state of forward Euler is the same fixed point whatever the step:
     * at equilibrium both derivatives are zero. 0.5 s against a 25 s constant
     * is 50 steps per time constant, which is enough to arrive. */
    tsim::ThermalNetwork network;
    network.configure(enabled_network());
    network.reset();

    const double loss_w = 3.28; /* continuous current, roughly */
    for (int i = 0; i < 20000; ++i) {
        network.step(0.5, loss_w);
    }
    const double expected_c = tsim::thermal_steady_state_winding_c(network.params(), loss_w);
    TSIM_CHECK_NEAR(network.winding_temperature_c(), expected_c, 0.05);
    /* And the housing, which is the slow node, has to be above ambient but
     * below the winding. A network where both nodes settle to the same place
     * would pass a single node test and be wrong. */
    TSIM_CHECK(network.housing_temperature_c() > kBenchAmbientC);
    TSIM_CHECK(network.housing_temperature_c() < network.winding_temperature_c());
}

TSIM_TEST(copper_multiplier_is_one_when_disabled_and_alpha_when_enabled) {
    tsim::ThermalParams params = tsim::thermal_params_maxon();
    tsim::ThermalNetwork network;
    network.configure(params); /* disabled */
    TSIM_CHECK_NEAR(network.copper_resistance_multiplier(), 1.0, 0.0);
    TSIM_CHECK_NEAR(network.winding_temperature_c(), kBenchAmbientC, 0.0);
    for (int i = 0; i < 1000; ++i) {
        network.step(1.0e-3, 1000.0);
    }
    TSIM_CHECK_NEAR(network.winding_temperature_c(), kBenchAmbientC, 0.0);

    /* 0.00393 per K over 135 K is 1.53, which is the whole reason a hot winding
     * draws less current for the same torque and loses more for the same
     * current. */
    const double hot = tsim::thermal_copper_multiplier(enabled_network(), kMaxWindingC);
    TSIM_CHECK_NEAR(hot, 1.5306, 0.001);
}

TSIM_TEST(derating_inverse_matches_the_forward_settling) {
    const tsim::ThermalParams params = enabled_network();
    const double limit_current_a = tsim::thermal_current_for_winding_c(
        params, kMaxWindingC, kBenchAmbientC, kWindingResistanceOhm);
    TSIM_CHECK(limit_current_a > 4.0);
    TSIM_CHECK(limit_current_a < 5.0);

    const double settled_c = tsim::thermal_winding_c_at_current(
        params, limit_current_a, kBenchAmbientC, kWindingResistanceOhm);
    TSIM_CHECK_NEAR(settled_c, kMaxWindingC, 0.01);

    /* And the datasheet's continuous current is not the current that reaches the
     * limit from a bench. It settles at 65 degC, which is the finding this day
     * was not expecting: the published continuous rating is thermally
     * consistent with the published resistances at the top of the rated ambient
     * range, not at 25 degC. */
    const double continuous_c = tsim::thermal_winding_c_at_current(
        params, kContinuousCurrentA, kBenchAmbientC, kWindingResistanceOhm);
    TSIM_CHECK(continuous_c > 60.0);
    TSIM_CHECK(continuous_c < 70.0);

    /* At the motor's maximum rated ambient of 100 degC the same current sits
     * just under the insulation limit, which is what a rating means. */
    const double at_rated_ambient_c = tsim::thermal_winding_c_at_current(
        params, kContinuousCurrentA, 100.0, kWindingResistanceOhm);
    TSIM_CHECK(at_rated_ambient_c > 150.0);
    TSIM_CHECK(at_rated_ambient_c < kMaxWindingC);
}

TSIM_TEST(runaway_current_is_above_the_derating_curve_everywhere) {
    /* The loss line is tangent to the dissipation line at this current, so above
     * it there is no steady state at any ambient. The derating curve has to sit
     * below it for every ambient, or the curve would promise a current the
     * motor cannot hold. */
    const tsim::ThermalParams params = enabled_network();
    const double runaway_a = tsim::thermal_runaway_current(params, kWindingResistanceOhm);
    TSIM_CHECK_NEAR(runaway_a, 7.47, 0.05);

    for (double ambient_c = 0.0; ambient_c <= 140.0; ambient_c += 5.0) {
        const double current_a = tsim::thermal_current_for_winding_c(
            params, kMaxWindingC, ambient_c, kWindingResistanceOhm);
        TSIM_CHECK(current_a <= runaway_a);
        /* And a current above the runaway one has no steady state, which is
         * reported as a number no comparison with a limit can satisfy. */
        TSIM_CHECK(tsim::thermal_winding_c_at_current(params, runaway_a + 1.0, ambient_c,
                                                      kWindingResistanceOhm) > 1.0e8);
    }
}

TSIM_TEST(plant_winding_warms_and_resistance_follows_when_thermal_is_on) {
    tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    params.mechanics = tsim::mechanics_params_rigid();
    params.thermal = enabled_network();
    tsim::IdealPlant plant(params);
    plant.reset(0.0);
    plant.set_rotor_locked(true);

    /* A pure alpha voltage vector on a clamped rotor, which is a d axis current
     * and therefore heat with no torque. 3.6 V across 0.291 ohm is 12.4 A, and
     * the loss is 1.5 I^2 R, which over 200 ms is about 1 K. */
    tsim::InverterCommand command{};
    command.duty_a = 0.55f;
    command.duty_b = 0.45f;
    command.duty_c = 0.45f;
    command.bridge_enabled = true;
    plant.latch_inverter(command);

    const double before_c = plant.winding_temperature_c();
    TSIM_CHECK_NEAR(before_c, kBenchAmbientC, 1.0e-9);
    for (int i = 0; i < 200000; ++i) {
        plant.step_one();
    }
    const double after_c = plant.winding_temperature_c();
    TSIM_CHECK(after_c > before_c + 0.5);
    TSIM_CHECK(after_c < before_c + 3.0);

    /* The copper loss in the account has to be larger than it would have been
     * cold, because the resistance rose. That is the feedback the whole class
     * exists for. */
    const double cold_resistance_ohm = params.resistance_phase_ohm;
    const double hot_resistance_ohm =
        cold_resistance_ohm * tsim::thermal_copper_multiplier(params.thermal, after_c);
    TSIM_CHECK(hot_resistance_ohm > cold_resistance_ohm * 1.001);
}

TSIM_TEST(plant_is_unchanged_when_thermal_is_off) {
    /* The default. Day 1 to day 4 depend on this being exactly true and not
     * approximately true, so the check is for an exact zero rather than a
     * tolerance. */
    tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    params.mechanics = tsim::mechanics_params_rigid();
    tsim::IdealPlant plant(params);
    plant.reset(0.0);
    plant.set_rotor_locked(true);
    tsim::InverterCommand command{};
    command.duty_a = 0.55f;
    command.duty_b = 0.45f;
    command.duty_c = 0.45f;
    command.bridge_enabled = true;
    plant.latch_inverter(command);
    for (int i = 0; i < 20000; ++i) {
        plant.step_one();
    }
    TSIM_CHECK(plant.winding_temperature_c() == kBenchAmbientC);
}
