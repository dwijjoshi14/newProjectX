# Firmware library used by the harness and the tests. Normal host flags.
add_library(tsim_firmware STATIC src/firmware.c)
target_include_directories(tsim_firmware PUBLIC include)
target_link_libraries(tsim_firmware PRIVATE tsim_warnings)

# Stub implementation of hal.h for host builds that do not run a plant.
add_library(tsim_hal_stub STATIC portable/stub_hal.c)
target_include_directories(tsim_hal_stub PUBLIC include portable)
target_link_libraries(tsim_hal_stub PRIVATE tsim_warnings)

# Compile only portability target. Compiled as if for a bare metal compiler,
# against the in repo stub of the device headers. Never flashed, never run in
# CI beyond the size check. A real board would add hal_stm32.c and change
# nothing under src/.
add_executable(firmware_portable src/firmware.c portable/stub_hal.c portable/stub_main.c)
target_include_directories(firmware_portable PRIVATE include portable)
target_compile_options(
  firmware_portable
  PRIVATE -ffreestanding -Wall -Wextra -Wpedantic -Wdouble-promotion -Werror)
