cmake_minimum_required(VERSION 3.0.2)
project(continuumrobot_hw)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES continuumrobot_hw
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_node
  src/test_node.cpp
  src/Kinematic_CX.cpp
)

target_link_libraries(test_node
  ${catkin_LIBRARIES}
)

