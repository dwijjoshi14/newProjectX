/* sim_loop.h, one control window of the deterministic harness.
 *
 * The ordering lives here and nowhere else, so the harness and the closed loop
 * tests cannot drift apart: duty from the last tick latches at the window
 * start, the plant integrates half a window, the converter latches currents at
 * the window centre, the plant integrates the rest, and the firmware tick runs.
 * Sample to actuation is 25 us to 75 us by construction.
 */
#ifndef TSIM_HARNESS_SIM_LOOP_H
#define TSIM_HARNESS_SIM_LOOP_H

#include <cstdint>

namespace tsim {
class IPlant;
}

/* Run one 50 us window. The caller owns the tick counter, so time is integer
 * and no clock is read. */
void sim_run_window(tsim::IPlant &plant, std::uint64_t tick);

#endif /* TSIM_HARNESS_SIM_LOOP_H */
