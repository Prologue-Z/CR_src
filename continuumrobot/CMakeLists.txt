cmake_minimum_required(VERSION 3.0.2)
project(continuumrobot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  msgs_continuumrobot
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES continuumrobot
   CATKIN_DEPENDS roscpp msgs_continuumrobot
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${PROJECT_SOURCE_DIR}/lib
  ${catkin_LIB_DIRS}
)

add_executable(${PROJECT_NAME}_node
	src/continuumrobot_node.cpp
	src/Function_Common.cpp
	src/Class_Motor.cpp
	src/Class_USBCAN.cpp
	src/Class_ContinuumRobot.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  -lpthread
  libECanVci.so.1
  -lusb
)
add_dependencies(${PROJECT_NAME}_node 
   msgs_continuumrobot_generate_messages_cpp
)


