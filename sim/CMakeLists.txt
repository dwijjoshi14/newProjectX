add_library(
  tsim_sim
  STATIC
  src/ideal_plant.cpp
  src/switching_plant.cpp
  src/mechanics.cpp
  src/thermal.cpp
  src/plant_params.cpp)
target_include_directories(tsim_sim PUBLIC include)
target_link_libraries(tsim_sim PRIVATE tsim_warnings)
