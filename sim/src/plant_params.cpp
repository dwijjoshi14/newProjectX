#include "tsim/plant_params.h"

namespace tsim {

PlantParams maxon_ec_i_40_70w_params() {
    PlantParams params{};
    /* maxon EC-i 40, 70 W, 36 V, part 496655. Source: maxongroup.com product
     * page and EC-i 40 catalogue page, retrieval date in docs/parameters.md. */
    params.pole_pairs = 7.0;
    params.resistance_phase_ohm = 0.291;           /* 0.582 ohm phase to phase, wye, halved */
    params.inductance_phase_h = 2.56e-4;           /* 0.512 mH phase to phase, halved */
    params.flux_linkage_wb = 4.390e-3;             /* Kt 46.1 mNm/A / (1.5 * 7 pole pairs) */
    params.inertia_kg_m2 = 2.3e-6;                 /* 23 gcm2, datasheet rotor inertia */
    params.viscous_friction_nm_s_per_rad = 1.0e-5; /* ASSUMED, see parameters.md */
    params.bus_voltage_v = 36.0;                   /* datasheet nominal voltage */
    /* Disabled, but filled in, so a plant that never asked for heat reports the
     * bench temperature rather than zero. A board with no thermistor reads
     * ambient, and that is the constant this one has to agree with. */
    params.thermal = thermal_params_maxon();
    return params;
}

/* One rigid body, no friction. This is the day 1 to day 3 mechanics and it
 * stays the default so that publishing a new plant does not quietly move a
 * number that is already in docs/results.md. */
MechanicalParams mechanics_params_rigid() {
    return MechanicalParams{};
}

/* The joint with a handle on it: a coupling that is not infinitely stiff and a
 * hand's worth of inertia on the far side. Shaft friction is on, because a
 * joint that has none cannot be held still, and holding still is most of what
 * a haptic joint does. */
MechanicalParams mechanics_params_compliant_load() {
    MechanicalParams mechanics{};
    mechanics.load_inertia_kg_m2 = 1.5e-4; /* ASSUMED, hand on a 30 mm knob */
    /* Derived on day 4, not assumed: a 6 mm steel shaft over 40 mm is
     * G J / L = 79.3e9 * 1.27e-10 / 0.04 = 250 N m/rad, and that is what holds a
     * knob on a motor. The 0.9 N m/rad this table used to carry put the load's
     * own resonance at 12 Hz, which caps the speed loop near 3 Hz on a joint
     * that has to feel like something solid. See params.md for the arithmetic
     * and the bring-up log for what the 0.9 did to the loop. */
    mechanics.coupling_stiffness_nm_per_rad = 250.0;
    /* A metal shaft over this length is barely damped: 2e-3 N m s/rad is a ratio
     * of 0.04 on the coupled mode. ASSUMED, because no coupling has been
     * measured, and the joint's own Coulomb friction is what actually damps it. */
    mechanics.coupling_damping_nm_s_per_rad = 2.0e-3;
    mechanics.coulomb_torque_nm = 8.0e-3; /* ASSUMED, bearing and seal */
    return mechanics;
}

/* The hand as well: a spring and a damper from the load to ground. The hand is
 * not a mass on a bearing, it is a compliant thing that pushes back. */
MechanicalParams mechanics_params_hand() {
    MechanicalParams mechanics = mechanics_params_compliant_load();
    mechanics.load_spring_nm_per_rad = 0.45;    /* ASSUMED, literature range for a hand */
    mechanics.load_damping_nm_s_per_rad = 0.02; /* ASSUMED, same source */
    return mechanics;
}

/* A wall at an angle, one sided, with no hand spring. Stiffness is 1.8 N m/rad
 * against a 46 mNm/A motor, so the joint deflects visibly into it instead of
 * being stopped dead by a number that is really an infinite constraint. */
MechanicalParams mechanics_params_wall(double wall_angle_rad) {
    MechanicalParams mechanics = mechanics_params_compliant_load();
    mechanics.wall_angle_rad = wall_angle_rad;
    mechanics.wall_stiffness_nm_per_rad = 1.8; /* ASSUMED, rigid wall on a 30 mm knob */
    /* A hand braced against a wall is still a hand, so it still damps. Without
     * this the load side has no dissipation at all and a rendered spring rings
     * through the coupling forever. */
    mechanics.load_damping_nm_s_per_rad = 0.02; /* ASSUMED, same row as the hand */
    return mechanics;
}

InverterParams inverter_params_ideal() {
    InverterParams inverter{};
    inverter.instantaneous_switching = false;
    inverter.dead_time_s = 0.0;
    inverter.on_resistance_ohm = 0.0;
    inverter.switching_crossover_s = 0.0;
    inverter.bus_capacitance_f = 0.0;
    inverter.bus_esr_ohm = 0.0;
    inverter.bus_source_resistance_ohm = 0.0;
    inverter.dead_time_linear_a = 0.0;
    return inverter;
}

InverterParams inverter_params_realistic() {
    InverterParams inverter{};
    inverter.instantaneous_switching = true;
    inverter.dead_time_s = 500.0e-9;          /* ASSUMED, parameters.md */
    inverter.on_resistance_ohm = 3.0e-3;      /* ASSUMED, one device */
    inverter.switching_crossover_s = 50.0e-9; /* ASSUMED, see parameters.md */
    inverter.bus_capacitance_f = 100.0e-6;    /* ASSUMED, parameters.md */
    inverter.bus_esr_ohm = 20.0e-3;           /* ASSUMED, parameters.md */
    inverter.bus_source_resistance_ohm = 0.1; /* ASSUMED, supply and cabling */
    inverter.dead_time_linear_a = 0.5;        /* ASSUMED, half the assumed ripple */
    return inverter;
}

/* Datasheet thermal values, with the two heat capacities derived from the two
 * published time constants: 24.9 s / 2.27 K/W = 11.0 J/K and
 * 1020 s / 8.17 K/W = 125 J/K. That derivation is why the time constants are
 * not free parameters: they are the same two numbers rearranged, and a test
 * checks the rearrangement.
 *
 * Off by default. A run that wants heat asks for it, which is what keeps every
 * day 1 to day 4 number exactly where it was. */
ThermalParams thermal_params_maxon() {
    ThermalParams params{};
    params.enabled = false;
    params.ambient_c = 25.0;                      /* ASSUMED, bench */
    params.winding_to_housing_k_per_w = 2.27;     /* datasheet */
    params.housing_to_ambient_k_per_w = 8.17;     /* datasheet */
    params.winding_heat_capacity_j_per_k = 11.0;  /* 24.9 s / 2.27 K/W */
    params.housing_heat_capacity_j_per_k = 125.0; /* 1020 s / 8.17 K/W */
    params.copper_alpha_per_k = 0.00393;          /* copper at 20 degC, IEC 60228 */
    params.copper_reference_c = 20.0;             /* the temperature alpha is quoted at */
    params.initial_winding_c = params.ambient_c;
    params.initial_housing_c = params.ambient_c;
    return params;
}

} // namespace tsim
