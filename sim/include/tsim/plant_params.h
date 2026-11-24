/* plant_params.h, the one place physical constants are bound to numbers.
 * Every value is cited or marked ASSUMED in docs/parameters.md. */
#ifndef TSIM_SIM_PLANT_PARAMS_H
#define TSIM_SIM_PLANT_PARAMS_H

#include "tsim/plant.h"

namespace tsim {

/* Maxon EC-i 40, 70 W, 36 V, part 496655, datasheet values with the
 * derivation recorded in docs/parameters.md. Friction is ASSUMED. */
PlantParams maxon_ec_i_40_70w_params();

/* The average voltage source of rung 1 and 2: no switching, no dead time, no
 * device resistance, a stiff bus. Reproduces day 1 and day 2 exactly. */
InverterParams inverter_params_ideal();

/* The inverter this project assumes: 500 ns dead time, 3 mOhm per device, a
 * 50 ns voltage and current overlap at each transition, a 100 uF link with
 * 20 mOhm ESR behind 0.1 ohm of source resistance, switched at 20 kHz. Every
 * value is ASSUMED in docs/parameters.md because no gate driver or device has
 * been selected. */
InverterParams inverter_params_realistic();

} // namespace tsim

#endif /* TSIM_SIM_PLANT_PARAMS_H */
