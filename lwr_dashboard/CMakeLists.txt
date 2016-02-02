cmake_minimum_required(VERSION 2.8.3)
project(lwr_dashboard)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  lwr_fri
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lwr_dashboard
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

