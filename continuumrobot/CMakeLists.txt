cmake_minimum_required(VERSION 3.0.2)
project(continuumrobot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  msgs_continuumrobot
  message_generation
)

catkin_package(
   INCLUDE_DIRS include
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

##datacollection_node
add_executable(datacollection_node
	src/datacollection_node.cpp
	src/Function_Common.cpp
	src/Class_Motor.cpp
	src/Class_USBCAN.cpp
	src/Class_ContinuumRobot.cpp
)

target_link_libraries(datacollection_node
  ${catkin_LIBRARIES}
  -lpthread
  libECanVci.so.1
  -lusb
)
add_dependencies(datacollection_node 
   msgs_continuumrobot_generate_messages_cpp
)
##debugging_node
add_executable(debugging_node
	src/debugging_node.cpp
	src/Function_Common.cpp
	src/Class_Motor.cpp
	src/Class_USBCAN.cpp
	src/Class_ContinuumRobot.cpp
)

target_link_libraries(debugging_node
  ${catkin_LIBRARIES}
  -lpthread
  libECanVci.so.1
  -lusb
)
add_dependencies(debugging_node
   msgs_continuumrobot_generate_messages_cpp
)
