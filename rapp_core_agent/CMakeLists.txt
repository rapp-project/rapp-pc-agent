cmake_minimum_required(VERSION 2.8.3)
project(rapp_core_agent)
set(ROS_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rapp_core_agent_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  roscpp
  rapp_core_agent_msgs
)


#catkin_install_python(
  #PROGRAMS scripts/
  #DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# install python modules under this package
catkin_python_setup()


# Core Agent Implementaiton

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(core_agent src/core_agent.cpp)

target_link_libraries(core_agent ${catkin_LIBRARIES})

add_dependencies(
  core_agent
  ${catkin_EXPORTED_TARGETS}
  ${rapp_core_agent_msgs_EXPORTED_TARGETS}
)
