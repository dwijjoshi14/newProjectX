/* mechanics.h, everything between the motor torque and the world: the shaft's
 * own friction, a coupling that can be compliant and can have lost motion, and
 * a load that may be a hand or a wall.
 *
 * Why this is a separate class rather than lines in each plant: the electrical
 * model is what changed on day 3, the mechanics is what changed on day 4, and
 * two plants must not disagree about inertia. One implementation, one set of
 * tests, both plants use it.
 *
 * The rigid configuration reproduces the day 1 to day 3 body exactly: one
 * inertia, viscous friction, an external torque. That is asserted, not
 * assumed, because day 1 to day 3 results are pinned and a refactor that moved
 * them would be a silent correction of published numbers.
 *
 * Frames: rotor angle and speed are rotor frame, load angle is the load side of
 * the coupling in the same rotor frame, so a rigid coupling means the two are
 * equal. Torques are N m, positive drives positive rotation on the rotor side
 * and positive load torque opposes it.
 */
#ifndef TSIM_SIM_MECHANICS_H
#define TSIM_SIM_MECHANICS_H

namespace tsim {

/* Every value here is cited or marked ASSUMED in docs/parameters.md. A zero
 * means the feature is absent, which is the only way this struct has a
 * meaningful default. */
struct MechanicalParams {
    double load_inertia_kg_m2;             /* hand side of the coupling, kg m^2, 0 = rigid */
    double coupling_stiffness_nm_per_rad;  /* 0 = rigid coupling, N m/rad */
    double coupling_damping_nm_s_per_rad;  /* N m s/rad */
    double backlash_rad;                   /* total lost motion, split half each side, rad */
    double load_spring_nm_per_rad;         /* the hand as a spring, N m/rad */
    double load_damping_nm_s_per_rad;      /* the hand as a damper, N m s/rad */
    double load_rest_angle_rad;            /* angle the hand spring is relaxed at, rad */
    double wall_angle_rad;                 /* one sided stop, engaged beyond this, rad */
    double wall_stiffness_nm_per_rad;      /* N m/rad, large is a rigid wall */
    double coulomb_torque_nm;              /* shaft friction, N m, 0 = none */
};

/* Result of one mechanical step. The rotor state is returned rather than kept,
 * because the plants already own it and hand it to the electrical model in the
 * same form they always did. */
struct MechanicsStep {
    double theta_rotor_rad;
    double omega_rotor_rad_s;
};

/* What the mechanics is doing, for logging and for tests. Angles and speeds as
 * above. The energies are the mechanical system's own account: viscous and
 * Coulomb are accumulated losses, elastic is what the springs hold right now. */
struct MechanicsReadout {
    double theta_load_rad;       /* load side angle, rad */
    double omega_load_rad_s;     /* load side speed, rad/s */
    double torque_coupling_nm;   /* rotor into the load, N m */
    double torque_hand_nm;       /* hand reaction on the load, N m */
    double torque_wall_nm;       /* wall reaction on the load, N m */
    double torque_coulomb_nm;    /* shaft friction this step, N m */
    double viscous_j;            /* accumulated viscous loss, J */
    double coulomb_j;            /* accumulated friction loss, J */
    double elastic_j;            /* stored in the springs at this instant, J */
};

class MechanicalSystem {
  public:
    MechanicalSystem();

    /* The two rotor constants come from PlantParams, which is where the motor's
     * own inertia and bearing drag live. They are passed at configure time
     * rather than copied into MechanicalParams so that there is one place a
     * motor constant is written down and no chance of the two disagreeing. */
    void configure(const MechanicalParams &params, double rotor_inertia_kg_m2,
                   double rotor_viscous_nm_s_per_rad);

    /* Relaxed coupling at this rotor angle, load at rest. */
    void reset(double theta_rotor_rad);

    /* One step. Forward Euler like the electrical model, at the plant step. The
     * rigid configuration does the same arithmetic the plants used to do
     * inline, in the same order, so published results do not move. */
    MechanicsStep step(double dt_s, double theta_rotor_rad, double omega_rotor_rad_s,
                       double torque_motor_nm, double torque_external_nm);

    const MechanicsReadout &readout() const;

    const MechanicalParams &params() const;

    /* True when the load is rigidly attached and the whole thing is one body. */
    bool rigid() const;

    /* Torque the coupling carries for a given relative angle and speed, with
     * the lost motion removed. Zero inside the backlash gap, which is what the
     * dead zone test asserts. Pure: it reads the parameters and nothing else. */
    double coupling_torque_nm(double delta_rad, double delta_omega_rad_s) const;

  private:
    MechanicalParams params_{};
    double rotor_inertia_kg_m2_ = 0.0;
    double rotor_viscous_nm_s_per_rad_ = 0.0;
    MechanicsReadout readout_{};
    double theta_load_rad_ = 0.0;
    double omega_load_rad_s_ = 0.0;
};

} // namespace tsim

#endif /* TSIM_SIM_MECHANICS_H */
