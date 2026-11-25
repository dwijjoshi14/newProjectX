add_executable(tsim_tests test_main.cpp test_plant.cpp test_inverter.cpp test_mechanics.cpp)
target_link_libraries(tsim_tests PRIVATE tsim_sim tsim_warnings)
add_test(NAME unit COMMAND tsim_tests)

add_executable(tsim_firmware_tests test_main.cpp test_firmware.cpp test_foc.cpp)
target_link_libraries(tsim_firmware_tests PRIVATE tsim_firmware tsim_hal_stub tsim_warnings)
add_test(NAME firmware COMMAND tsim_firmware_tests)

add_executable(tsim_control_tests test_main.cpp test_control.cpp test_sensing.cpp)
target_link_libraries(tsim_control_tests PRIVATE tsim_sim_hal tsim_sim tsim_firmware tsim_warnings)
add_test(NAME control COMMAND tsim_control_tests)

add_test(
  NAME deterministic_repeat
  COMMAND sh ${CMAKE_SOURCE_DIR}/tools/check_repeatable_run.sh $<TARGET_FILE:tsim_harness>
          speed_step)

add_test(
  NAME firmware_boundary
  COMMAND sh ${CMAKE_SOURCE_DIR}/tools/check_firmware_boundary.sh ${CMAKE_SOURCE_DIR}/firmware
          $<TARGET_OBJECTS:firmware_portable>)

find_package(Python3 COMPONENTS Interpreter QUIET)
if(TSIM_SANITIZERS)
  # Sanitizer instrumentation inflates .text by several kB, so the budget only
  # applies to a build that could ship.
  message(STATUS "sanitizer build, the firmware size budget is not enforced")
elseif(Python3_Interpreter_FOUND)
  add_test(
    NAME size_budget
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/check_size.py
            $<TARGET_FILE:firmware_portable> ${CMAKE_SOURCE_DIR}/firmware/firmware_size_baseline.txt)
else()
  message(WARNING "python3 not found, the size budget test is skipped")
endif()
