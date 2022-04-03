cmake_minimum_required(VERSION 3.0.2)
project(continuumrobot_digitaltwin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES continuumrobot_digitaltwin
 CATKIN_DEPENDS roscpp
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(node_Draw
 src/node_Draw.cpp
)

add_executable(node_TfBroadcaster
 src/node_TfBroadcaster.cpp
 src/class_continuumrobot.cpp
)


target_link_libraries(node_Draw
  ${catkin_LIBRARIES}
)

target_link_libraries(node_TfBroadcaster
  ${catkin_LIBRARIES}
)