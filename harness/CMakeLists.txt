# The harness side of hal.h and the control window, shared by the harness
# binary and the closed loop tests so both run the same timing.
add_library(tsim_sim_hal STATIC src/sim_hal.cpp src/sim_loop.cpp)
target_include_directories(tsim_sim_hal PUBLIC include)
target_link_libraries(tsim_sim_hal PUBLIC tsim_firmware PRIVATE tsim_sim tsim_warnings)

add_executable(tsim_harness src/main.cpp)
target_link_libraries(tsim_harness PRIVATE tsim_sim_hal tsim_sim tsim_firmware tsim_warnings)
