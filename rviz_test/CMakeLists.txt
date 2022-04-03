cmake_minimum_required(VERSION 3.0.2)
project(rviz_test)

<<<<<<< HEAD
<<<<<<< HEAD

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES continuumrobot_hw
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(node_test
  src/node_test.cpp
  src/class_continuumrobot.cpp
)

add_executable(tf_broadcaster
 src/tf_broadcaster.cpp
 )


target_link_libraries(node_test
  ${catkin_LIBRARIES}
)

target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
