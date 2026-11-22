/* plant_params.h, the one place physical constants are bound to numbers.
 * Every value is cited or marked ASSUMED in docs/parameters.md. */
#ifndef TSIM_SIM_PLANT_PARAMS_H
#define TSIM_SIM_PLANT_PARAMS_H

#include "tsim/plant.h"

namespace tsim {

/* Maxon EC-i 40, 70 W, 36 V, part 496655, datasheet values with the
 * derivation recorded in docs/parameters.md. Friction is ASSUMED. */
PlantParams maxon_ec_i_40_70w_params();

} // namespace tsim

#endif /* TSIM_SIM_PLANT_PARAMS_H */
