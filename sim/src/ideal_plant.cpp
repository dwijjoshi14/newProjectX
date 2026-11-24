#include "tsim/ideal_plant.h"

#include <cmath>

namespace tsim {
namespace {

/* Clarke transform, phase to alpha, A or V. Amplitude invariant. */
double clarke_alpha(double a, double b, double c) {
    return (2.0 / 3.0) * (a - 0.5 * b - 0.5 * c);
}

/* Clarke transform, phase to beta, A or V. Amplitude invariant. */
double clarke_beta(double a, double b, double c) {
    (void)a;
    return (2.0 / 3.0) * (std::sqrt(3.0) / 2.0) * (b - c);
}

} // namespace

IdealPlant::IdealPlant(const PlantParams &params) : params_(params) {}

void IdealPlant::configure(const PlantParams &params) {
    params_ = params;
}

void IdealPlant::reset(double theta_rotor_rad) {
    ia_a_ = 0.0;
    ib_a_ = 0.0;
    ic_a_ = 0.0;
    theta_rotor_rad_ = theta_rotor_rad;
    omega_rotor_rad_s_ = 0.0;
    torque_motor_nm_ = 0.0;
    torque_external_nm_ = 0.0;
    phase_voltage_a_v_ = 0.0;
    phase_voltage_b_v_ = 0.0;
    phase_voltage_c_v_ = 0.0;
    p_electrical_w_ = 0.0;
    p_copper_w_ = 0.0;
    p_mechanical_w_ = 0.0;
    steps_ = 0;
    energy_ = EnergyAccount{};
}

void IdealPlant::latch_inverter(const InverterCommand &command) {
    command_ = command;
}

void IdealPlant::step_one() {
    if (!command_.bridge_enabled) {
        /* Open windings: no current can flow, so no torque and no copper loss. */
        ia_a_ = 0.0;
        ib_a_ = 0.0;
        ic_a_ = 0.0;
        phase_voltage_a_v_ = 0.0;
        phase_voltage_b_v_ = 0.0;
        phase_voltage_c_v_ = 0.0;
        torque_motor_nm_ = 0.0;
        p_electrical_w_ = 0.0;
        p_copper_w_ = 0.0;
    } else {
        /* Average inverter model: duty sets the phase voltage, common mode removed
         * the way a floating wye neutral removes it. */
        const double duty_mean = (command_.duty_a + command_.duty_b + command_.duty_c) / 3.0;
        phase_voltage_a_v_ = params_.bus_voltage_v * (command_.duty_a - duty_mean);
        phase_voltage_b_v_ = params_.bus_voltage_v * (command_.duty_b - duty_mean);
        phase_voltage_c_v_ = params_.bus_voltage_v * (command_.duty_c - duty_mean);

        const double theta_e_rad = theta_rotor_rad_ * params_.pole_pairs;
        const double emf_peak_v = params_.flux_linkage_wb * params_.pole_pairs * omega_rotor_rad_s_;
        const double e_a = -emf_peak_v * std::sin(theta_e_rad);
        const double e_b = -emf_peak_v * std::sin(theta_e_rad - kTwoPiOverThree);
        const double e_c = -emf_peak_v * std::sin(theta_e_rad + kTwoPiOverThree);

        const double di_a = (phase_voltage_a_v_ - params_.resistance_phase_ohm * ia_a_ - e_a) /
                            params_.inductance_phase_h;
        const double di_b = (phase_voltage_b_v_ - params_.resistance_phase_ohm * ib_a_ - e_b) /
                            params_.inductance_phase_h;
        const double di_c = (phase_voltage_c_v_ - params_.resistance_phase_ohm * ic_a_ - e_c) /
                            params_.inductance_phase_h;

        ia_a_ += di_a * step_s_;
        ib_a_ += di_b * step_s_;
        ic_a_ += di_c * step_s_;

        /* Torque from the q axis current, amplitude invariant: T = 1.5 p lambda iq. */
        const double i_alpha_a = clarke_alpha(ia_a_, ib_a_, ic_a_);
        const double i_beta_a = clarke_beta(ia_a_, ib_a_, ic_a_);
        const double iq_a = -i_alpha_a * std::sin(theta_e_rad) + i_beta_a * std::cos(theta_e_rad);
        torque_motor_nm_ = 1.5 * params_.pole_pairs * params_.flux_linkage_wb * iq_a;

        p_electrical_w_ =
            phase_voltage_a_v_ * ia_a_ + phase_voltage_b_v_ * ib_a_ + phase_voltage_c_v_ * ic_a_;
        p_copper_w_ =
            params_.resistance_phase_ohm * (ia_a_ * ia_a_ + ib_a_ * ib_a_ + ic_a_ * ic_a_);
        energy_.electrical_j += p_electrical_w_ * step_s_;
        energy_.copper_j += p_copper_w_ * step_s_;
    }

    p_mechanical_w_ = torque_motor_nm_ * omega_rotor_rad_s_;
    energy_.mechanical_j += p_mechanical_w_ * step_s_;

    if (locked_) {
        /* Clamped rotor: the torque is reacted by the clamp, so speed and angle
         * stay put and no mechanical work crosses the shaft. */
        omega_rotor_rad_s_ = 0.0;
        steps_ += 1;
        return;
    }

    const double p_viscous_w =
        params_.viscous_friction_nm_s_per_rad * omega_rotor_rad_s_ * omega_rotor_rad_s_;
    energy_.viscous_j += p_viscous_w * step_s_;

    const double torque_net_nm = torque_motor_nm_ - torque_external_nm_ -
                                 params_.viscous_friction_nm_s_per_rad * omega_rotor_rad_s_;
    omega_rotor_rad_s_ += (torque_net_nm / params_.inertia_kg_m2) * step_s_;
    theta_rotor_rad_ += omega_rotor_rad_s_ * step_s_;
    steps_ += 1;
}

std::uint64_t IdealPlant::step_count() const {
    return steps_;
}

double IdealPlant::time_s() const {
    return static_cast<double>(steps_) * step_s_;
}

ElectricalReadout IdealPlant::sample_electrical() const {
    return ElectricalReadout{ia_a_,
                             ib_a_,
                             ic_a_,
                             phase_voltage_a_v_,
                             phase_voltage_b_v_,
                             phase_voltage_c_v_,
                             params_.bus_voltage_v};
}

MechanicalState IdealPlant::mechanical() const {
    return MechanicalState{theta_rotor_rad_, omega_rotor_rad_s_, torque_motor_nm_,
                           torque_external_nm_};
}

Telemetry IdealPlant::telemetry() const {
    return Telemetry{p_electrical_w_, p_copper_w_, p_mechanical_w_, torque_motor_nm_};
}

EnergyAccount IdealPlant::energy() const {
    return energy_;
}

void IdealPlant::set_external_torque_nm(double torque_nm) {
    torque_external_nm_ = torque_nm;
}

void IdealPlant::set_rotor_locked(bool locked) {
    locked_ = locked;
}

void IdealPlant::set_step_ns_for_convergence(std::uint64_t step_ns) {
    if (step_ns == 0) {
        return;
    }
    step_s_ = 1.0e-9 * static_cast<double>(step_ns);
}

const PlantParams &IdealPlant::params() const {
    return params_;
}

} // namespace tsim
