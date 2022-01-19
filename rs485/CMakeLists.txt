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


target_link_libraries(rs485_node
   ${catkin_LIBRARIES}
)
add_dependencies(rs485_node
   msgs_continuumrobot_generate_messages_cpp
)

###########delete after test
add_executable(topic_test_node
      src/topic_test_node.cpp
)
target_link_libraries(topic_test_node
   ${catkin_LIBRARIES}
)
add_dependencies(topic_test_node
   msgs_continuumrobot_generate_messages_cpp
)
