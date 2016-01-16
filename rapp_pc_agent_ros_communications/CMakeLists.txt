cmake_minimum_required(VERSION 2.8.3)
project(rapp_pc_agent_ros_communications)
set(ROS_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  std_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_generation
    message_runtime
    std_msgs
)

