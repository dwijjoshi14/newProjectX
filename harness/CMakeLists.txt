# The harness side of hal.h and the control window, shared by the harness
# binary and the closed loop tests so both run the same timing.
add_library(tsim_sim_hal STATIC src/sim_hal.cpp src/sim_loop.cpp src/link.cpp)
target_include_directories(tsim_sim_hal PUBLIC include)
target_link_libraries(tsim_sim_hal PUBLIC tsim_firmware PRIVATE tsim_sim tsim_warnings)

# The packet two joints send each other, as bytes. Portable, so the codec tests
# run everywhere the control tests do.
add_library(tsim_tandem_wire STATIC src/tandem_wire.cpp)
target_include_directories(tsim_tandem_wire PUBLIC include)
target_link_libraries(tsim_tandem_wire PRIVATE tsim_warnings)

add_executable(tsim_harness src/main.cpp)
target_link_libraries(tsim_harness PRIVATE tsim_sim_hal tsim_sim tsim_firmware tsim_warnings)

# Opens the loop and records one switching period at 1 us, for the sampling
# instant figure. Needs the firmware to settle the current first, and nothing
# else.
add_executable(tsim_sample_probe src/sample_probe.cpp)
target_link_libraries(tsim_sample_probe PRIVATE tsim_sim_hal tsim_sim tsim_firmware tsim_warnings)

# Two joints over a delayed link, with the passivity observer on or off. This is
# the stability boundary: how stiff a rendered coupling a given delay allows.
add_executable(tsim_bilateral src/bilateral.cpp)
target_link_libraries(tsim_bilateral PRIVATE tsim_sim_hal tsim_sim tsim_firmware tsim_warnings)

# Efficiency over a torque and speed grid, at a stated winding temperature.
add_executable(tsim_efficiency src/efficiency.cpp)
target_link_libraries(tsim_efficiency PRIVATE tsim_sim_hal tsim_sim tsim_firmware tsim_warnings)

# The thermal network on its own, because the housing's 1020 s constant cannot
# be soaked through a 1 MHz plant inside one session.
add_executable(tsim_thermal src/thermal_tool.cpp)
target_link_libraries(tsim_thermal PRIVATE tsim_sim tsim_warnings)

# Two drives, two processes, one real socket between them. POSIX only, because a
# datagram socket on this project means BSD sockets and both the runner and the
# sandbox are Linux.
if(UNIX)
  add_library(tsim_tandem_link STATIC src/udp_link.cpp)
  target_include_directories(tsim_tandem_link PUBLIC include)
  target_link_libraries(tsim_tandem_link PUBLIC tsim_tandem_wire PRIVATE tsim_warnings)

  add_executable(tsim_tandem_node src/tandem_node.cpp)
  target_link_libraries(
    tsim_tandem_node PRIVATE tsim_tandem_link tsim_sim_hal tsim_sim tsim_firmware tsim_warnings)
endif()
