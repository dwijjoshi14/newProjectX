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

} // namespace tsim
