cmake_minimum_required(VERSION 2.8.3)
project(rapp_pc_core_agent)
set(ROS_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS
  rapp_pc_agent_ros_communications
)


catkin_package(
  CATKIN_DEPENDS
  rapp_pc_agent_ros_communications
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

#catkin_install_python(
  #PROGRAMS scripts/
  #DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# install python modules under this package
catkin_python_setup()
