cmake_minimum_required(VERSION 2.8.3)
project(grobot_utilities)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    gazebo_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  DEPENDS
    gazebo_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(set_initial_pose
  src/set_initial_pose_node.cpp
)
target_link_libraries(set_initial_pose ${catkin_LIBRARIES})

#############
## Install ##
#############

install(
  TARGETS
    set_initial_pose
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
