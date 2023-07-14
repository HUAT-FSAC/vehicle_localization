cmake_minimum_required(VERSION 3.0.2)
project(vehicle_localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/3rd/GeographicLib/include/
  ${YAML_CPP_INCLUDEDIR}
)

# YAML support
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
list(APPEND ALL_TARGET_LIBRARIES ${YAML_CPP_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

add_library(DEPS SHARED
  src/eskf.cpp
  src/filter_flow.cpp
  src/gps_flow.cpp
  src/filter_flow.cpp
  3rd/GeographicLib/src/LocalCartesian.cpp
  3rd/GeographicLib/src/Geocentric.cpp
  3rd/GeographicLib/src/Math.cpp
)

find_package(rostime REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(vehicle_localization src/main.cpp src/filter_flow.cpp)

target_link_libraries(DEPS
    ${ALL_TARGET_LIBRARIES}
)

target_link_libraries(vehicle_localization ${catkin_LIBRARIES})
target_link_libraries(vehicle_localization DEPS)

install(FILES
  ${header_file}
  DESTINATION ${CATKIN_DEVEL_PREFIX}/include
)

include_directories(
  include
  /home/neko/Code/dev_ws/devel/include
  ${catkin_INCLUDE_DIRS}
  /opt/ros/melodic/include
)