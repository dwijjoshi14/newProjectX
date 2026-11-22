cmake_minimum_required(VERSION 3.22)

project(
  tandem_sim
  VERSION 0.1.0
  DESCRIPTION "Switching level digital twin of a haptic robot joint"
  LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(TSIM_BUILD_TESTS "Build the host test suite" ON)
option(TSIM_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)
option(TSIM_SANITIZERS "Sanitizer build, the size budget does not apply" OFF)

add_library(tsim_warnings INTERFACE)
if(MSVC)
  target_compile_options(tsim_warnings INTERFACE /W4)
else()
  target_compile_options(
    tsim_warnings
    INTERFACE -Wall -Wextra -Wpedantic -Wshadow -Wundef -Wcast-qual -Wpointer-arith
              $<$<COMPILE_LANGUAGE:C>:-Wstrict-prototypes>)
endif()

if(TSIM_WARNINGS_AS_ERRORS)
  if(MSVC)
    target_compile_options(tsim_warnings INTERFACE /WX)
  else()
    target_compile_options(tsim_warnings INTERFACE -Werror)
  endif()
endif()

add_subdirectory(firmware)
add_subdirectory(sim)
add_subdirectory(harness)

if(TSIM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
