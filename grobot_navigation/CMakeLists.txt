cmake_minimum_required(VERSION 2.8.3)
project(grobot_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    fiducial_msgs
    move_base_msgs
    std_srvs
    actionlib
    tf
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  DEPENDS
    fiducial_msgs
    move_base_msgs
    std_srvs
    actionlib
    tf
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(follow_marker 
  src/marker_follower.cpp
  src/follow_marker_node.cpp
)
target_link_libraries(follow_marker ${catkin_LIBRARIES})

#############
## Install ##
#############

install(
  TARGETS
    follow_marker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
