cmake_minimum_required(VERSION 3.10)
project(unit_sim)

set(CMAKE_CXX_STANDARD 11)

find_package(yaml-cpp REQUIRED)

include_directories(
        ../
)


#add_executable(unit_sim ../AUVDynamics.hpp ../Sensor.hpp ../AttitudeController.hpp ../PositionController.hpp
#         ../AUVDynamics.cpp ../Sensor.cpp ../AttitudeController.cpp ../PositionController.cpp)
set(CMAKE_BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Install)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(UNDERWATER_SIMULATION_DIRECTORY $ENV{HOME}/UWSim/src/underwater_simulation)

set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR})

add_executable(unit_sim ../AUVDynamics.hpp unit_sim.hpp
        ../AUVDynamics.cpp unit_sim.cpp)

target_link_libraries(unit_sim yaml-cpp)

set_target_properties(unit_sim
        PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
        )

install(TARGETS unit_sim DESTINATION rel CONFIGURATIONS Release)
install(TARGETS unit_sim DESTINATION dbg CONFIGURATIONS Debug)

install(FILES
        ${UNDERWATER_SIMULATION_DIRECTORY}/uwsim_dynamics/config/dynamics_dolphin.yaml
        ${UNDERWATER_SIMULATION_DIRECTORY}/uwsim_sensor/config/uwsim_sensor.yaml
        DESTINATION ${CMAKE_BINARY_DIR}/config COMPONENT config)