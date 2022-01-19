cmake_minimum_required(VERSION 3.0.2)
project(rs485)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  msgs_continuumrobot
  std_msgs
  message_generation
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rs485
   CATKIN_DEPENDS roscpp serial msgs_continuumrobot
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(rs485_node
		src/rs485_node.cpp
		src/Class_ForceSensor.cpp
		src/Function_Commom.cpp
)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_node 
   msgs_continuumrobot_generate_messages_cpp
)



