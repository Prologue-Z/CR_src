cmake_minimum_required(VERSION 3.0.2)
project(msgs_continuumrobot)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  Msg_Force.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
)

