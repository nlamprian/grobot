#include "grobot_utilities/initial_pose_setter.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "set_initial_pose");
  InitialPoseSetter pose_setter;
  ros::spin();
  return 0;
}
