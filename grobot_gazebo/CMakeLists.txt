cmake_minimum_required(VERSION 2.8.3)
project(grobot_gazebo)

find_package(catkin REQUIRED
  COMPONENTS
    gazebo_ros
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    gazebo_ros
)

#############
## Install ##
#############

install(DIRECTORY media models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
