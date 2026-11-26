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

/* Run one 50 us window on a joint with no drive: the bridge is held open by
 * default, so no current flows and the only torque is whatever the caller put
 * in with set_external_torque_nm. This is the far side of a bilateral pair, and
 * it exists because hal.h is a single MCU contract: two drives are two
 * processes, and this one process can only host one. What the far side keeps is
 * the mechanics, which is what the link is coupled to. */
void sim_run_remote_window(tsim::IPlant &plant);

#endif /* TSIM_HARNESS_SIM_LOOP_H */
