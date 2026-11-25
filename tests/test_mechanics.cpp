/* Mechanics tests, written before the mechanics existed as a class.
 *
 * The order matters: the first test pins the rigid configuration to the day 1
 * body, because that is the one thing a refactor of published results must not
 * change. Everything after it is new behaviour that has an analytic answer, so
 * "the two mass resonates near 100 Hz" is checked against sqrt(k (1/J1 + 1/J2))
 * and not against a number somebody liked.
 */

#include <cmath>
#include <vector>

#include "tsim/ideal_plant.h"
#include "tsim/mechanics.h"
#include "tsim/plant.h"
#include "tsim/plant_params.h"
#include "tsim_test.h"

namespace {

constexpr double kRotorInertiaKgM2 = 2.3e-6;
constexpr double kHandInertiaKgM2 = 1.5e-4;
constexpr double kCouplingNmPerRad = 0.9;

double kinetic_j(double inertia, double omega) {
    return 0.5 * inertia * omega * omega;
}

/* Count sign changes of a signal, which for an oscillation it starts away from
 * zero is two per period. */
std::size_t zero_crossings(const std::vector<double> &values) {
    std::size_t crossings = 0;
    for (std::size_t i = 1; i < values.size(); ++i) {
        if ((values[i - 1] < 0.0 && values[i] >= 0.0) || (values[i - 1] > 0.0 && values[i] <= 0.0)) {
            crossings += 1;
        }
    }
    return crossings;
}

} // namespace

TSIM_TEST(rigid_mechanics_reproduce_the_day_one_body) {
    /* Day 1 wrote three lines inline: viscous drag from the speed, a net torque
     * divided by the inertia, and a first order angle update. The mechanics
     * class has to produce the same trajectory to the last bit and this test
     * asks it to within a floating point tolerance that is tight enough to
     * catch a reordered sum. */
    tsim::PlantParams params = tsim::maxon_ec_i_40_70w_params();
    params.mechanics = tsim::mechanics_params_rigid();
    tsim::IdealPlant plant(params);
    plant.reset(0.0);

    /* Negative external torque accelerates positive rotation, which is the
     * convention the coast down test already uses. */
    plant.set_external_torque_nm(-0.05);
    const std::uint32_t steps = 10000; /* 10 ms */
    for (std::uint32_t i = 0; i < steps; ++i) {
        plant.step_one();
    }

    const double time_s = static_cast<double>(steps) * tsim::kPlantStepS;
    const double tau_s =
        params.inertia_kg_m2 / params.viscous_friction_nm_s_per_rad;
    const double terminal_rad_s = 0.05 / params.viscous_friction_nm_s_per_rad;
    const double expected_rad_s = terminal_rad_s * (1.0 - std::exp(-time_s / tau_s));

    /* The angle is the integral of that, which is not the triangular
     * approximation: theta = terminal * (t - tau (1 - exp(-t/tau))). */
    const double expected_theta_rad =
        terminal_rad_s * (time_s - tau_s * (1.0 - std::exp(-time_s / tau_s)));
    TSIM_CHECK_NEAR(plant.mechanical().omega_rotor_rad_s, expected_rad_s, 1.0e-4);
    TSIM_CHECK_NEAR(plant.mechanical().theta_rotor_rad, expected_theta_rad, 2.0e-3);

    /* The rigid configuration reports the load as the rotor, and stores no
     * elastic energy, because there is no spring to store any. */
    TSIM_CHECK(plant.mechanical().theta_load_rad == plant.mechanical().theta_rotor_rad);
    TSIM_CHECK(plant.mechanical().torque_coupling_nm == 0.0);
    TSIM_CHECK(plant.energy().elastic_j == 0.0);
    TSIM_CHECK(plant.energy().coulomb_j == 0.0);

    /* Work in equals kinetic energy out plus what the drag took. */
    const double kinetic_j =
        0.5 * params.inertia_kg_m2 * expected_rad_s * expected_rad_s;
    const double work_j = 0.05 * plant.mechanical().theta_rotor_rad;
    TSIM_CHECK_NEAR(work_j, kinetic_j + plant.energy().viscous_j, 2.0e-3);
}

TSIM_TEST(two_masses_resonate_where_the_analytic_frequency_says) {
    /* Two inertias joined by one spring, nothing else. The relative angle
     * oscillates at sqrt(k (1/J1 + 1/J2)), which for this joint and this hand is
     * 630 rad/s, 100 Hz, the number docs/parameters.md quotes for the coupling.
     * Measuring the crossings rather than fitting anything keeps the test to
     * arithmetic a reader can redo. */
    tsim::MechanicalParams mechanics{};
    mechanics.load_inertia_kg_m2 = kHandInertiaKgM2;
    mechanics.coupling_stiffness_nm_per_rad = kCouplingNmPerRad;
    mechanics.coupling_damping_nm_s_per_rad = 0.0;

    tsim::MechanicalSystem system;
    system.configure(mechanics, kRotorInertiaKgM2, 0.0);
    system.reset(0.0);

    tsim::MechanicsStep state{0.01, 0.0};
    const std::uint32_t steps = 500000; /* 500 ms, about fifty periods */
    std::vector<double> delta;
    delta.reserve(steps);
    for (std::uint32_t i = 0; i < steps; ++i) {
        state = system.step(tsim::kPlantStepS, state.theta_rotor_rad, state.omega_rotor_rad_s, 0.0,
                            0.0);
        delta.push_back(state.theta_rotor_rad - system.readout().theta_load_rad);
    }

    const std::size_t crossings = zero_crossings(delta);
    TSIM_CHECK(crossings > 90);
    const double time_s = static_cast<double>(steps) * tsim::kPlantStepS;
    const double measured_rad_s =
        tsim::kTwoPi * static_cast<double>(crossings) / (2.0 * time_s);
    const double analytic_rad_s = std::sqrt(kCouplingNmPerRad * (1.0 / kRotorInertiaKgM2 +
                                                                1.0 / kHandInertiaKgM2));
    TSIM_CHECK_NEAR(measured_rad_s, analytic_rad_s, 0.01 * analytic_rad_s);

    /* No net torque on the pair, so the centre of mass does not move. Momentum
     * is the invariant a sign error in the coupling reaction breaks. */
    TSIM_CHECK_NEAR(kRotorInertiaKgM2 * state.omega_rotor_rad_s +
                        kHandInertiaKgM2 * system.readout().omega_load_rad_s,
                    0.0, 1.0e-9);
}

TSIM_TEST(an_undamped_two_mass_keeps_its_energy) {
    tsim::MechanicalParams mechanics{};
    mechanics.load_inertia_kg_m2 = kHandInertiaKgM2;
    mechanics.coupling_stiffness_nm_per_rad = kCouplingNmPerRad;

    tsim::MechanicalSystem system;
    system.configure(mechanics, kRotorInertiaKgM2, 0.0);
    system.reset(0.0);

    tsim::MechanicsStep state{0.01, 0.0};
    const double initial_j = 0.5 * kCouplingNmPerRad * 0.01 * 0.01;

    double worst_j = 0.0;
    for (std::uint32_t i = 0; i < 50000; ++i) {
        state = system.step(tsim::kPlantStepS, state.theta_rotor_rad, state.omega_rotor_rad_s, 0.0,
                            0.0);
        const double delta_rad = state.theta_rotor_rad - system.readout().theta_load_rad;
        const double total_j = kinetic_j(kRotorInertiaKgM2, state.omega_rotor_rad_s) +
                               kinetic_j(kHandInertiaKgM2, system.readout().omega_load_rad_s) +
                               0.5 * kCouplingNmPerRad * delta_rad * delta_rad;
        worst_j = std::fmax(worst_j, std::fabs(total_j - initial_j));
    }

    /* Semi implicit Euler is symplectic, so what it does to the energy is a
     * bounded oscillation of order dt times the frequency, 6e-4 here, and not a
     * drift. Both are checked: the excursion, and that fifty periods later the
     * energy is back where it started rather than lower. */
    const double delta_rad = state.theta_rotor_rad - system.readout().theta_load_rad;
    const double final_j = kinetic_j(kRotorInertiaKgM2, state.omega_rotor_rad_s) +
                           kinetic_j(kHandInertiaKgM2, system.readout().omega_load_rad_s) +
                           0.5 * kCouplingNmPerRad * delta_rad * delta_rad;
    TSIM_CHECK(worst_j < 2.0e-3 * initial_j);
    TSIM_CHECK_NEAR(final_j, initial_j, 2.0e-3 * initial_j);
}

TSIM_TEST(coulomb_friction_holds_the_shaft_below_breakaway) {
    /* A joint that cannot hold still is not a haptic joint. Below the breakaway
     * torque the shaft must not move at all, not creep, and the test asserts
     * exactly zero rather than a small number. */
    tsim::MechanicalParams mechanics{};
    mechanics.coulomb_torque_nm = 8.0e-3;

    tsim::MechanicalSystem system;
    system.configure(mechanics, kRotorInertiaKgM2, 0.0);
    system.reset(0.0);

    tsim::MechanicsStep state{0.0, 0.0};
    for (std::uint32_t i = 0; i < 100000; ++i) { /* 100 ms */
        state = system.step(tsim::kPlantStepS, state.theta_rotor_rad, state.omega_rotor_rad_s,
                            5.0e-3, 0.0);
    }
    TSIM_CHECK(state.theta_rotor_rad == 0.0);
    TSIM_CHECK(state.omega_rotor_rad_s == 0.0);
    TSIM_CHECK_NEAR(system.readout().torque_coulomb_nm, -5.0e-3, 1.0e-12);
    TSIM_CHECK(system.readout().coulomb_j == 0.0);

    /* Above breakaway the net torque is the excess and nothing else. */
    const double drive_nm = 12.0e-3;
    for (std::uint32_t i = 0; i < 10000; ++i) { /* 10 ms */
        state = system.step(tsim::kPlantStepS, state.theta_rotor_rad, state.omega_rotor_rad_s,
                            drive_nm, 0.0);
    }
    const double expected_rad_s = (drive_nm - mechanics.coulomb_torque_nm) / kRotorInertiaKgM2 *
                                  10.0e-3;
    TSIM_CHECK_NEAR(state.omega_rotor_rad_s, expected_rad_s, 1.0e-3 * expected_rad_s);
}

TSIM_TEST(a_coasting_shaft_stops_in_finite_time) {
    /* Viscous drag alone gives an exponential that never reaches zero and a
     * joint that never quite stops. Coulomb friction reaches zero in finite
     * time and then holds, which is what the test asserts, including that the
     * angle stops changing afterwards. */
    tsim::MechanicalParams mechanics{};
    mechanics.coulomb_torque_nm = 8.0e-3;

    tsim::MechanicalSystem system;
    system.configure(mechanics, kRotorInertiaKgM2, 0.0);
    system.reset(0.0);

    tsim::MechanicsStep state{0.0, 0.0};
    for (std::uint32_t i = 0; i < 2000; ++i) { /* 2 ms spin up */
        state = system.step(tsim::kPlantStepS, state.theta_rotor_rad, state.omega_rotor_rad_s,
                            0.05, 0.0);
    }
    TSIM_CHECK(state.omega_rotor_rad_s > 30.0);

    std::uint32_t steps_to_stop = 0;
    const std::uint32_t limit = 100000;
    while (state.omega_rotor_rad_s != 0.0 && steps_to_stop < limit) {
        state = system.step(tsim::kPlantStepS, state.theta_rotor_rad, state.omega_rotor_rad_s, 0.0,
                            0.0);
        steps_to_stop += 1;
    }
    TSIM_CHECK(steps_to_stop < limit);
    /* 43 rad/s against 8 mNm on 2.3e-6 kg m^2 is 3478 rad/s^2, so 12 ms. */
    TSIM_CHECK(steps_to_stop > 5000);
    TSIM_CHECK(steps_to_stop < 20000);

    const double stopped_rad = state.theta_rotor_rad;
    for (std::uint32_t i = 0; i < 100000; ++i) { /* 100 ms at rest */
        state = system.step(tsim::kPlantStepS, state.theta_rotor_rad, state.omega_rotor_rad_s, 0.0,
                            0.0);
    }
    TSIM_CHECK(state.omega_rotor_rad_s == 0.0);
    TSIM_CHECK(state.theta_rotor_rad == stopped_rad);
}

TSIM_TEST(backlash_is_a_dead_zone_of_exactly_the_stated_width) {
    tsim::MechanicalParams mechanics{};
    mechanics.load_inertia_kg_m2 = kHandInertiaKgM2;
    mechanics.coupling_stiffness_nm_per_rad = kCouplingNmPerRad;
    mechanics.backlash_rad = 0.01;

    tsim::MechanicalSystem system;
    system.configure(mechanics, kRotorInertiaKgM2, 0.0);

    /* The pure function first: no torque inside the gap, and outside it the
     * deflection measured from the edge, so the dead zone width is the backlash
     * and not something the load sets. */
    TSIM_CHECK(system.coupling_torque_nm(0.0049, 0.0) == 0.0);
    TSIM_CHECK(system.coupling_torque_nm(-0.0049, 0.0) == 0.0);
    TSIM_CHECK_NEAR(system.coupling_torque_nm(0.0051, 0.0), kCouplingNmPerRad * 1.0e-4, 1.0e-12);
    TSIM_CHECK_NEAR(system.coupling_torque_nm(-0.1, 0.0), -kCouplingNmPerRad * 0.095, 1.0e-12);

    /* Then the integrated version: with the load held by a stiff hand spring,
     * the rotor travels the half gap before the load feels anything. */
    mechanics.load_spring_nm_per_rad = 50.0;
    system.configure(mechanics, kRotorInertiaKgM2, 0.0);
    system.reset(0.0);

    tsim::MechanicsStep state{0.0, 0.0};
    double angle_at_engagement_rad = -1.0;
    for (std::uint32_t i = 0; i < 20000; ++i) { /* 20 ms */
        const double before = system.readout().torque_coupling_nm;
        state = system.step(tsim::kPlantStepS, state.theta_rotor_rad, state.omega_rotor_rad_s,
                            0.01, 0.0);
        if (before == 0.0 && system.readout().torque_coupling_nm != 0.0 &&
            angle_at_engagement_rad < 0.0) {
            angle_at_engagement_rad = state.theta_rotor_rad;
        }
        if (angle_at_engagement_rad < 0.0) {
            TSIM_CHECK(system.readout().theta_load_rad == 0.0);
        }
    }
    TSIM_CHECK(angle_at_engagement_rad > 0.0);
    TSIM_CHECK_NEAR(angle_at_engagement_rad, 0.005, 5.0e-4);
    TSIM_CHECK(system.readout().theta_load_rad > 0.0);
}

TSIM_TEST(a_wall_and_a_hand_hold_the_joint_at_their_own_angles) {
    /* Rigid coupling so the static answer is one line of arithmetic: motor
     * torque equals the reaction at the contact. */
    tsim::MechanicalParams wall{};
    wall.wall_angle_rad = 0.1;
    wall.wall_stiffness_nm_per_rad = 1.8;
    wall.load_damping_nm_s_per_rad = 0.02;

    tsim::MechanicalSystem system;
    system.configure(wall, kRotorInertiaKgM2, 0.0);
    system.reset(0.0);

    /* 300 ms, because the hand damping of 0.02 N m s/rad limits the approach to
     * 1 rad/s and the wall is 0.1 rad away: the joint takes a tenth of a second
     * to get there, which is a property of the joint and not of the test. */
    tsim::MechanicsStep state{0.0, 0.0};
    for (std::uint32_t i = 0; i < 300000; ++i) {
        state = system.step(tsim::kPlantStepS, state.theta_rotor_rad, state.omega_rotor_rad_s,
                            0.02, 0.0);
    }
    /* 0.02 N m into 1.8 N m/rad past 0.1 rad is 0.0111 rad of penetration. */
    TSIM_CHECK_NEAR(state.theta_rotor_rad, 0.1111, 2.0e-4);
    TSIM_CHECK_NEAR(system.readout().torque_wall_nm, 0.02, 2.0e-3);

    tsim::MechanicalParams hand{};
    hand.load_spring_nm_per_rad = 0.45;
    hand.load_damping_nm_s_per_rad = 0.02;
    system.configure(hand, kRotorInertiaKgM2, 0.0);
    system.reset(0.0);

    state = tsim::MechanicsStep{0.0, 0.0};
    for (std::uint32_t i = 0; i < 300000; ++i) {
        state = system.step(tsim::kPlantStepS, state.theta_rotor_rad, state.omega_rotor_rad_s,
                            0.02, 0.0);
    }
    /* 0.02 N m into 0.45 N m/rad is 0.0444 rad, and the hand holds it there. */
    TSIM_CHECK_NEAR(state.theta_rotor_rad, 0.04444, 2.0e-4);
    TSIM_CHECK_NEAR(system.readout().torque_hand_nm, 0.02, 2.0e-3);
    TSIM_CHECK(system.readout().torque_wall_nm == 0.0);
}
