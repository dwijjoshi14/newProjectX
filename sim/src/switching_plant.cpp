#include "tsim/switching_plant.h"

#include <algorithm>
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

/* Sign of the current, with the dead time error faded out below linear_a. The
 * leg spends part of each dead time freewheeling in each direction once the
 * ripple crosses zero inside the switching period, so the average error is
 * smaller than the full sign would give. Above linear_a the sign is single
 * valued and the full error applies. */
double dead_time_sign(double current_a, double linear_a) {
    if (!(linear_a > 0.0)) {
        return (current_a > 0.0) ? 1.0 : ((current_a < 0.0) ? -1.0 : 0.0);
    }
    const double normalised = current_a / linear_a;
    if (normalised > 1.0) {
        return 1.0;
    }
    if (normalised < -1.0) {
        return -1.0;
    }
    return normalised;
}

} // namespace

SwitchingPlant::SwitchingPlant(const PlantParams &params, const InverterParams &inverter)
    : params_(params), inverter_(inverter) {
    mechanics_.configure(params_.mechanics, params_.inertia_kg_m2,
                         params_.viscous_friction_nm_s_per_rad);
}

void SwitchingPlant::configure(const PlantParams &params) {
    params_ = params;
    mechanics_.configure(params_.mechanics, params_.inertia_kg_m2,
                         params_.viscous_friction_nm_s_per_rad);
}

void SwitchingPlant::reset(double theta_rotor_rad) {
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
    bus_cap_v_ = params_.bus_voltage_v;
    bus_terminal_v_ = params_.bus_voltage_v;
    bus_current_a_ = 0.0;
    switching_power_w_ = 0.0;
    p_electrical_w_ = 0.0;
    p_copper_w_ = 0.0;
    p_mechanical_w_ = 0.0;
    steps_ = 0;
    energy_ = EnergyAccount{};
    mechanics_.reset(theta_rotor_rad);
}

void SwitchingPlant::latch_inverter(const InverterCommand &command) {
    command_ = command;
}

/* Fill the three phase voltages for this step and the DC current the bridge
 * draws, then advance the capacitor. Called from step_one. */
void SwitchingPlant::step_inverter() {
    const double duty[3] = {static_cast<double>(command_.duty_a),
                            static_cast<double>(command_.duty_b),
                            static_cast<double>(command_.duty_c)};
    const double current_a[3] = {ia_a_, ib_a_, ic_a_};

    /* The switching period is the control period: these drives run the carrier
     * synchronous with the current loop, and the harness already treats one
     * window as one switching period. */
    const std::uint64_t window_steps =
        static_cast<std::uint64_t>(std::llround(kControlPeriodS / step_s_));
    const std::uint64_t phase = steps_ % window_steps;

    double leg[3] = {0.0, 0.0, 0.0};
    if (inverter_.instantaneous_switching) {
        /* Centre aligned carrier peaking at the window centre. The high time of
         * a leg is the interval ((1 - d) / 2, (1 + d) / 2) of the period, which
         * is symmetric about the centre and leaves the current ripple symmetric
         * about the ADC instant.
         *
         * The edge is placed inside the step rather than snapped to it. A 1 us
         * step can only put an edge on a 1 us grid, which over a 50 us period is
         * a 2 percent duty error, and at 36 V that is 0.72 V: twice the dead
         * time error this model exists to study. Averaging the fraction of each
         * step the leg is high removes that quantisation and costs one min and
         * one max per leg. It does mean the current ripple is piecewise linear
         * at the step size rather than continuous through the edge, which is
         * the honest limit of a 1 MHz plant. */
        const double step_start = static_cast<double>(phase) / static_cast<double>(window_steps);
        const double step_end = static_cast<double>(phase + 1u) / static_cast<double>(window_steps);
        for (int k = 0; k < 3; ++k) {
            const double high_start = 0.5 * (1.0 - duty[k]);
            const double high_end = 0.5 * (1.0 + duty[k]);
            const double overlap = std::min(high_end, step_end) - std::max(high_start, step_start);
            leg[k] =
                (overlap > 0.0) ? std::min(overlap * static_cast<double>(window_steps), 1.0) : 0.0;
        }
    } else {
        leg[0] = duty[0];
        leg[1] = duty[1];
        leg[2] = duty[2];
    }

    /* Switching loss: during each transition the device carries the leg current
     * while the voltage commutes, so the energy is the overlap area of a
     * triangle, half of bus times current times the crossover time. A leg that
     * commutates does it twice per period and each commutation turns one device
     * off and the other on, so four device events per leg, which is
     * 2 * bus * current * crossover for the leg.
     *
     * The loss is allocated per step rather than dumped at the window boundary,
     * because a 50 ns transition that lands on one microsecond step is a spike
     * of tens of watts and a bus current that no hardware could draw. Spreading
     * it over the period keeps the energy exact and the DC current physical. */
    switching_power_w_ = 0.0;
    for (int k = 0; k < 3; ++k) {
        if (duty[k] > 0.0 && duty[k] < 1.0) {
            const double magnitude_a = std::fabs(current_a[k]);
            switching_power_w_ += 2.0 * bus_terminal_v_ * magnitude_a *
                                  inverter_.switching_crossover_s / kControlPeriodS;
        }
    }

    /* Dead time as a fraction of the switching period: the volt seconds lost
     * per leg per period, which is the standard average error model. */
    const double dead_time_duty =
        (inverter_.dead_time_s > 0.0) ? inverter_.dead_time_s / kControlPeriodS : 0.0;

    double v_leg[3] = {0.0, 0.0, 0.0};
    for (int k = 0; k < 3; ++k) {
        const bool commutates = (duty[k] > 0.0 && duty[k] < 1.0);
        const double dead_time_v =
            commutates ? dead_time_sign(current_a[k], inverter_.dead_time_linear_a) *
                             bus_terminal_v_ * dead_time_duty
                       : 0.0;
        v_leg[k] =
            leg[k] * bus_terminal_v_ - inverter_.on_resistance_ohm * current_a[k] - dead_time_v;
    }

    /* Floating wye: only the differential part of the leg voltages reaches the
     * winding. This is also what makes the dead time error show up as
     * distortion instead of disappearing: three equal errors cancel, three
     * unequal ones do not. */
    const double mean_v = (v_leg[0] + v_leg[1] + v_leg[2]) / 3.0;
    phase_voltage_a_v_ = v_leg[0] - mean_v;
    phase_voltage_b_v_ = v_leg[1] - mean_v;
    phase_voltage_c_v_ = v_leg[2] - mean_v;

    /* DC current: the sum of the legs that are high, plus the switching loss. */
    const double loss_current_a =
        (bus_terminal_v_ > 1.0) ? switching_power_w_ / bus_terminal_v_ : 0.0;
    const double dc_current_a = leg[0] * ia_a_ + leg[1] * ib_a_ + leg[2] * ic_a_ + loss_current_a;
    bus_current_a_ = dc_current_a;

    if (inverter_.bus_capacitance_f > 0.0) {
        const double source_resistance_ohm = (inverter_.bus_source_resistance_ohm > 0.0)
                                                 ? inverter_.bus_source_resistance_ohm
                                                 : 1.0e-3;
        const double source_current_a =
            (params_.bus_voltage_v - bus_terminal_v_) / source_resistance_ohm;
        bus_cap_v_ += (source_current_a - dc_current_a) / inverter_.bus_capacitance_f * step_s_;
        if (bus_cap_v_ < 0.0) {
            bus_cap_v_ = 0.0;
        }
        bus_terminal_v_ = bus_cap_v_ - dc_current_a * inverter_.bus_esr_ohm;
        if (bus_terminal_v_ < 0.0) {
            bus_terminal_v_ = 0.0;
        }
    } else {
        bus_terminal_v_ = params_.bus_voltage_v;
    }
}

void SwitchingPlant::step_one() {
    if (!command_.bridge_enabled) {
        ia_a_ = 0.0;
        ib_a_ = 0.0;
        ic_a_ = 0.0;
        phase_voltage_a_v_ = 0.0;
        phase_voltage_b_v_ = 0.0;
        phase_voltage_c_v_ = 0.0;
        torque_motor_nm_ = 0.0;
        p_electrical_w_ = 0.0;
        p_copper_w_ = 0.0;
        switching_power_w_ = 0.0;
        bus_current_a_ = 0.0;
        if (inverter_.bus_capacitance_f > 0.0) {
            const double source_resistance_ohm = (inverter_.bus_source_resistance_ohm > 0.0)
                                                     ? inverter_.bus_source_resistance_ohm
                                                     : 1.0e-3;
            const double source_current_a =
                (params_.bus_voltage_v - bus_terminal_v_) / source_resistance_ohm;
            bus_cap_v_ += source_current_a / inverter_.bus_capacitance_f * step_s_;
            bus_terminal_v_ = bus_cap_v_;
        } else {
            bus_terminal_v_ = params_.bus_voltage_v;
        }
    } else {
        step_inverter();

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

        const double i_alpha_a = clarke_alpha(ia_a_, ib_a_, ic_a_);
        const double i_beta_a = clarke_beta(ia_a_, ib_a_, ic_a_);
        const double iq_a = -i_alpha_a * std::sin(theta_e_rad) + i_beta_a * std::cos(theta_e_rad);
        torque_motor_nm_ = 1.5 * params_.pole_pairs * params_.flux_linkage_wb * iq_a;

        /* Electrical power at the terminals, which includes whatever the dead
         * time and the device resistance drop. Copper loss is the winding's
         * share; the difference is the inverter's conduction loss. */
        p_electrical_w_ = bus_terminal_v_ * bus_current_a_;
        p_copper_w_ =
            params_.resistance_phase_ohm * (ia_a_ * ia_a_ + ib_a_ * ib_a_ + ic_a_ * ic_a_);
        energy_.electrical_j += p_electrical_w_ * step_s_;
        energy_.copper_j += p_copper_w_ * step_s_;
        energy_.switching_j += switching_power_w_ * step_s_;
    }

    p_mechanical_w_ = torque_motor_nm_ * omega_rotor_rad_s_;
    energy_.mechanical_j += p_mechanical_w_ * step_s_;

    if (locked_) {
        omega_rotor_rad_s_ = 0.0;
        steps_ += 1;
        return;
    }

    /* Same mechanics call as the ideal plant. A switching inverter and an
     * average one have to agree about what the shaft does, or a day 4 result on
     * one cannot be compared with a day 3 result on the other. */
    const MechanicsStep step = mechanics_.step(step_s_, theta_rotor_rad_, omega_rotor_rad_s_,
                                               torque_motor_nm_, torque_external_nm_);
    theta_rotor_rad_ = step.theta_rotor_rad;
    omega_rotor_rad_s_ = step.omega_rotor_rad_s;
    energy_.viscous_j = mechanics_.readout().viscous_j;
    energy_.coulomb_j = mechanics_.readout().coulomb_j;
    energy_.elastic_j = mechanics_.readout().elastic_j;
    steps_ += 1;
}

std::uint64_t SwitchingPlant::step_count() const {
    return steps_;
}

double SwitchingPlant::time_s() const {
    return static_cast<double>(steps_) * step_s_;
}

ElectricalReadout SwitchingPlant::sample_electrical() const {
    return ElectricalReadout{
        ia_a_,          ib_a_, ic_a_, phase_voltage_a_v_, phase_voltage_b_v_, phase_voltage_c_v_,
        bus_terminal_v_};
}

MechanicalState SwitchingPlant::mechanical() const {
    const MechanicsReadout &readout = mechanics_.readout();
    const double theta_load_rad = locked_ ? theta_rotor_rad_ : readout.theta_load_rad;
    const double omega_load_rad_s = locked_ ? 0.0 : readout.omega_load_rad_s;
    return MechanicalState{
        theta_rotor_rad_,           omega_rotor_rad_s_,     torque_motor_nm_,
        torque_external_nm_,        theta_load_rad,         omega_load_rad_s,
        readout.torque_coupling_nm, readout.torque_hand_nm, readout.torque_wall_nm,
        readout.torque_coulomb_nm};
}

Telemetry SwitchingPlant::telemetry() const {
    return Telemetry{p_electrical_w_, p_copper_w_, p_mechanical_w_, torque_motor_nm_};
}

EnergyAccount SwitchingPlant::energy() const {
    return energy_;
}

void SwitchingPlant::set_external_torque_nm(double torque_nm) {
    torque_external_nm_ = torque_nm;
}

void SwitchingPlant::set_rotor_locked(bool locked) {
    locked_ = locked;
}

void SwitchingPlant::set_step_ns_for_convergence(std::uint64_t step_ns) {
    if (step_ns == 0) {
        return;
    }
    step_s_ = 1.0e-9 * static_cast<double>(step_ns);
}

const PlantParams &SwitchingPlant::params() const {
    return params_;
}

const InverterParams &SwitchingPlant::inverter() const {
    return inverter_;
}

} // namespace tsim
