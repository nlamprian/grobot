cmake_minimum_required(VERSION 2.8.3)
project(grobot_maps)

find_package(catkin REQUIRED
  COMPONENTS
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
)

#############
## Install ##
#############

install(DIRECTORY maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
