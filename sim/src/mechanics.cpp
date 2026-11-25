#include "tsim/mechanics.h"

#include <algorithm>
#include <cmath>

namespace tsim {
namespace {

/* Something to compare a speed against, since a double that should be zero
 * after a friction stop is not always exactly zero. This is a numerical
 * epsilon, not a physical parameter: it is twelve orders below the speeds this
 * joint runs at and only exists so that the stiction test does not depend on
 * the last bit of an addition. */
constexpr double kStictionSpeedRadS = 1.0e-12;

} // namespace

MechanicalSystem::MechanicalSystem() {
    configure(MechanicalParams{}, 1.0, 0.0);
}

void MechanicalSystem::configure(const MechanicalParams &params, double rotor_inertia_kg_m2,
                                 double rotor_viscous_nm_s_per_rad) {
    params_ = params;
    rotor_inertia_kg_m2_ = rotor_inertia_kg_m2;
    rotor_viscous_nm_s_per_rad_ = rotor_viscous_nm_s_per_rad;
    /* The readout carries accumulated losses, so a reconfigure starts them at
     * zero rather than carrying the previous run's total into this one. */
    readout_ = MechanicsReadout{};
}

bool MechanicalSystem::rigid() const {
    return !(params_.coupling_stiffness_nm_per_rad > 0.0 && params_.load_inertia_kg_m2 > 0.0);
}

void MechanicalSystem::reset(double theta_rotor_rad) {
    theta_load_rad_ = theta_rotor_rad;
    omega_load_rad_s_ = 0.0;
    readout_ = MechanicsReadout{};
}

/* Coupling torque with lost motion. Inside the gap the teeth are not touching,
 * so neither the spring nor the damper acts. Outside it the deflection is
 * measured from the edge of the gap, which is what makes the lost motion
 * exactly backlash_rad wide instead of something that depends on the load. */
double MechanicalSystem::coupling_torque_nm(double delta_rad, double delta_omega_rad_s) const {
    const double half_gap_rad = 0.5 * params_.backlash_rad;
    const double magnitude_rad = std::fabs(delta_rad);
    if (magnitude_rad <= half_gap_rad) {
        return 0.0;
    }
    const double direction = (delta_rad > 0.0) ? 1.0 : -1.0;
    const double deflection_rad = (magnitude_rad - half_gap_rad) * direction;
    return params_.coupling_stiffness_nm_per_rad * deflection_rad +
           params_.coupling_damping_nm_s_per_rad * delta_omega_rad_s;
}

MechanicsStep MechanicalSystem::step(double dt_s, double theta_rotor_rad, double omega_rotor_rad_s,
                                     double torque_motor_nm, double torque_external_nm) {
    MechanicsStep out{theta_rotor_rad, omega_rotor_rad_s};

    /* Friction torque to apply this step, and whether it holds the shaft. The
     * rules are the discrete form of Coulomb plus stiction: at rest, friction
     * cancels whatever torque is smaller than breakaway and the shaft does not
     * move at all; in motion it opposes the velocity; and a step that would
     * carry the speed through zero stops at zero rather than crossing it. */
    auto friction_for = [this](double omega_rad_s, double drive_torque_nm, double *friction_nm) {
        const double breakaway_nm = params_.coulomb_torque_nm;
        if (!(breakaway_nm > 0.0)) {
            *friction_nm = 0.0;
            return false;
        }
        if (std::fabs(omega_rad_s) <= kStictionSpeedRadS) {
            if (std::fabs(drive_torque_nm) <= breakaway_nm) {
                *friction_nm = -drive_torque_nm;
                return true;
            }
            *friction_nm = (drive_torque_nm > 0.0) ? -breakaway_nm : breakaway_nm;
            return false;
        }
        *friction_nm = (omega_rad_s > 0.0) ? -breakaway_nm : breakaway_nm;
        return false;
    };

    /* Load side first: the reaction of whatever the joint is touching, which is
     * what the joint is for. Both terms are zero in a free run. With a rigid
     * coupling the load is the rotor, so the hand and the wall act on the rotor
     * angle directly. */
    const double theta_load_rad = rigid() ? theta_rotor_rad : theta_load_rad_;
    const double omega_load_rad_s = rigid() ? omega_rotor_rad_s : omega_load_rad_s_;
    readout_.torque_hand_nm =
        params_.load_spring_nm_per_rad * (theta_load_rad - params_.load_rest_angle_rad) +
        params_.load_damping_nm_s_per_rad * omega_load_rad_s;
    readout_.torque_wall_nm =
        (params_.wall_stiffness_nm_per_rad > 0.0 && theta_load_rad > params_.wall_angle_rad)
            ? params_.wall_stiffness_nm_per_rad * (theta_load_rad - params_.wall_angle_rad)
            : 0.0;

    const double viscous_nm = rotor_viscous_nm_s_per_rad_ * omega_rotor_rad_s;
    /* The no-reversal rule below belongs to Coulomb friction and to nothing
     * else. Applying it without friction clamps every speed zero crossing to
     * zero, which quietly destroys the momentum and the energy of a freely
     * swinging two mass body, and does it twice per period so it does not look
     * like a bug in the trace. The energy test caught it. */
    const bool coulomb_active = params_.coulomb_torque_nm > 0.0;
    /* The loss is accumulated at the speed the step started from, which is the
     * speed the drag torque was computed at. It is the same arithmetic day 1
     * wrote inline, so the published losses do not move by a last bit. */
    const double viscous_j_increment = viscous_nm * omega_rotor_rad_s * dt_s;

    double friction_nm = 0.0;
    if (rigid()) {
        const double total_inertia_kg_m2 = rotor_inertia_kg_m2_ + params_.load_inertia_kg_m2;
        const double drive_nm = torque_motor_nm - torque_external_nm - viscous_nm -
                                readout_.torque_hand_nm - readout_.torque_wall_nm;
        const bool held = friction_for(omega_rotor_rad_s, drive_nm, &friction_nm);
        double omega_rad_s = omega_rotor_rad_s;
        if (held) {
            omega_rad_s = 0.0;
        } else {
            const double acceleration = (drive_nm + friction_nm) / total_inertia_kg_m2;
            const double next_rad_s = omega_rad_s + acceleration * dt_s;
            /* Coulomb friction cannot reverse the shaft, so a step that would
             * take the speed through zero ends at zero, and the next step starts
             * from rest where the stiction rule decides whether it moves. */
            omega_rad_s = (coulomb_active && omega_rad_s * next_rad_s < 0.0) ? 0.0 : next_rad_s;
        }
        out.omega_rotor_rad_s = omega_rad_s;
        out.theta_rotor_rad = theta_rotor_rad + omega_rad_s * dt_s;

        readout_.theta_load_rad = out.theta_rotor_rad;
        readout_.omega_load_rad_s = omega_rad_s;
        readout_.torque_coupling_nm = 0.0;
        readout_.torque_coulomb_nm = friction_nm;
        readout_.viscous_j += viscous_j_increment;
        readout_.coulomb_j += std::fabs(friction_nm * omega_rad_s) * dt_s;

        const double hand_deflection_rad = out.theta_rotor_rad - params_.load_rest_angle_rad;
        const double wall_deflection_rad =
            std::max(0.0, out.theta_rotor_rad - params_.wall_angle_rad);
        readout_.elastic_j =
            0.5 * params_.load_spring_nm_per_rad * hand_deflection_rad * hand_deflection_rad +
            0.5 * params_.wall_stiffness_nm_per_rad * wall_deflection_rad * wall_deflection_rad;
        return out;
    }

    /* Two masses. The coupling carries what the relative angle and speed say,
     * with the lost motion removed, and it is the only path between them. */
    const double delta_rad = theta_rotor_rad - theta_load_rad_;
    const double coupling_nm = coupling_torque_nm(delta_rad, omega_rotor_rad_s - omega_load_rad_s_);

    const double drive_nm = torque_motor_nm - torque_external_nm - viscous_nm - coupling_nm;
    const bool held = friction_for(omega_rotor_rad_s, drive_nm, &friction_nm);
    double omega_rotor_rad_s_new = omega_rotor_rad_s;
    if (held) {
        omega_rotor_rad_s_new = 0.0;
    } else {
        const double next_rad_s =
            omega_rotor_rad_s + (drive_nm + friction_nm) / rotor_inertia_kg_m2_ * dt_s;
        omega_rotor_rad_s_new =
            (coulomb_active && omega_rotor_rad_s * next_rad_s < 0.0) ? 0.0 : next_rad_s;
    }
    out.omega_rotor_rad_s = omega_rotor_rad_s_new;
    out.theta_rotor_rad = theta_rotor_rad + omega_rotor_rad_s_new * dt_s;

    const double load_acceleration =
        (coupling_nm - readout_.torque_hand_nm - readout_.torque_wall_nm) /
        params_.load_inertia_kg_m2;
    omega_load_rad_s_ += load_acceleration * dt_s;
    theta_load_rad_ += omega_load_rad_s_ * dt_s;

    readout_.theta_load_rad = theta_load_rad_;
    readout_.omega_load_rad_s = omega_load_rad_s_;
    readout_.torque_coupling_nm = coupling_nm;
    readout_.torque_coulomb_nm = friction_nm;
    readout_.viscous_j += viscous_j_increment + params_.load_damping_nm_s_per_rad *
                                                    omega_load_rad_s_ * omega_load_rad_s_ * dt_s;
    readout_.coulomb_j += std::fabs(friction_nm * omega_rotor_rad_s_new) * dt_s;

    const double deflection_rad = std::max(0.0, std::fabs(out.theta_rotor_rad - theta_load_rad_) -
                                                    0.5 * params_.backlash_rad);
    const double hand_deflection_rad = theta_load_rad_ - params_.load_rest_angle_rad;
    const double wall_deflection_rad = std::max(0.0, theta_load_rad_ - params_.wall_angle_rad);
    readout_.elastic_j =
        0.5 * params_.coupling_stiffness_nm_per_rad * deflection_rad * deflection_rad +
        0.5 * params_.load_spring_nm_per_rad * hand_deflection_rad * hand_deflection_rad +
        0.5 * params_.wall_stiffness_nm_per_rad * wall_deflection_rad * wall_deflection_rad;
    return out;
}

const MechanicsReadout &MechanicalSystem::readout() const {
    return readout_;
}

const MechanicalParams &MechanicalSystem::params() const {
    return params_;
}

} // namespace tsim
