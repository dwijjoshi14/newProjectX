/* Entry point for the compile only portable target. It exists so the firmware
 * is linked, not just compiled, and so the size budget has an artefact. */

#include <stdint.h>

#include "firmware.h"

#define STUB_MAIN_TICKS 1000u /* control ticks before exit, count */

int main(void) {
    firmware_init();
    for (uint32_t i = 0u; i < STUB_MAIN_TICKS; ++i) {
        firmware_tick();
    }
    return (firmware_status()->ticks == STUB_MAIN_TICKS) ? 0 : 1;
}
