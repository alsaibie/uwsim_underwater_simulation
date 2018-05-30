cmake_minimum_required(VERSION 2.8.3)
project(uwsim_dynamics)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs nav_msgs geometry_msgs sensor_msgs tf tf_conversions)
set(UWSIM_MSGS_PATH /home/alsaibie/UWSim/install_isolated/)

find_package(uwsim_msgs PATHS ${UWSIM_MSGS_PATH})

find_package(Eigen3 3.3 REQUIRED)

#include(../tools/eigen/Eigen/)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS rospy std_msgs nav_msgs geometry_msgs sensor_msgs tf tf_conversions message_runtime uwsim_msgs
        DEPENDS EIGEN3
        #python_orocos_kdl
)

include_directories(
        include
        ../dynamics_stack/
        ${catkin_INCLUDE_DIRS}
        ${uwsim_msgs_INCLUDE_DIRS}
#        ${Eigen_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}  include/dynamics_node.hpp ../dynamics_stack/AUVDynamics.hpp
                                src/dynamics_node.cpp ../dynamics_stack/AUVDynamics.cpp)


target_link_libraries(
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${uwsim_msgs_LIBRARIES}
        ${EIGEN3_DIR}
        )

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

#install(PROGRAMS src/dynamics.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#install(PROGRAMS src/dynamics_dolphin.py src/tests/dynamics_dolphin_test.py src/tests/dynamics_dolphin_test_display.py
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS src/tests/dynamics_dolphin_test.py src/tests/dynamics_dolphin_test_display.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)
