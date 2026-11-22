add_executable(tsim_harness src/main.cpp src/sim_hal.cpp)
target_include_directories(tsim_harness PRIVATE include)
target_link_libraries(tsim_harness PRIVATE tsim_sim tsim_firmware tsim_warnings)
