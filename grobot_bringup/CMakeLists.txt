cmake_minimum_required(VERSION 2.8.3)
project(grobot_bringup)

find_package(catkin REQUIRED
  COMPONENTS
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    navigation
    gmapping
    turtlebot3_bringup
    turtlebot3_slam
    turtlebot3_example
    grobot_gazebo
    grobot_maps
    grobot_utilities
    grobot_navigation
    aruco_detect
)

catkin_add_env_hooks(
    model
    SHELLS bash zsh
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks
)

#############
## Install ##
#############

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
