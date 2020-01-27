#include <grobot_navigation/marker_follower.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "marker_follower");
  MarkerFollower follower;
  ros::spin();
  return 0;
}
