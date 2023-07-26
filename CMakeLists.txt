cmake_minimum_required(VERSION 3.0.2)
project(gps_imu_fusion)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  common_msgs
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gps_imu_fusion
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
  /home/neko/Code/dev_ws/src/gps_imu_fusion/include/gps_imu_fusion
  #include
  ${catkin_INCLUDE_DIRS}
)

# catkin_package()

add_executable(msg src/msg_handler.cpp)
add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES})
target_link_libraries(msg ${catkin_LIBRARIES})