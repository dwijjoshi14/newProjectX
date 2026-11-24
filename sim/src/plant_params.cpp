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
    return params;
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

} // namespace tsim
