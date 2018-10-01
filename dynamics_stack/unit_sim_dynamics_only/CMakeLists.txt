cmake_minimum_required(VERSION 3.10)
project(unit_sim_dynamics_only)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_STACK_SIZE "10000000")
find_package(yaml-cpp REQUIRED)

include_directories(
        ../
        ../tools/
        ../tools/matrix/matrix
)

set(CMAKE_BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Install)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(UNDERWATER_SIMULATION_DIRECTORY $ENV{HOME}/UWSim/src/underwater_simulation)

set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR})

add_executable(unit_sim_dynamics_only
        ../AUVDynamics.hpp
        ../Sensor.hpp
        ../PositionController.hpp
        ../AttitudeController.hpp
        unit_sim.hpp
        ../AUVDynamics.cpp
        ../Sensor.cpp
        ../PositionController.cpp
        ../AttitudeController.cpp
        ../tools/mathlib/math/filter/LowPassFilter2p.cpp
        unit_sim.cpp main.cpp)

target_link_libraries(unit_sim_dynamics_only yaml-cpp)

set_target_properties(unit_sim_dynamics_only
        PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
        )

install(TARGETS unit_sim_dynamics_only DESTINATION rel CONFIGURATIONS Release)
install(TARGETS unit_sim_dynamics_only DESTINATION dbg CONFIGURATIONS Debug)

# If I want to install yaml config files. Let me edit them directly for now.

#install(FILES
#        ${UNDERWATER_SIMULATION_DIRECTORY}/uwsim_dynamics/config/dynamics_dolphin.yaml
#        ${UNDERWATER_SIMULATION_DIRECTORY}/uwsim_sensor/config/uwsim_sensor.yaml
#        ../attitude_controller.yaml
#        ../position_controller.yaml
#        DESTINATION ${CMAKE_BINARY_DIR}/config COMPONENT config)
