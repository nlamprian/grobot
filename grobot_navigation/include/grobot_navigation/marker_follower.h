#ifndef GROBOT_NAVIGATION_MARKER_FOLLOWER_H
#define GROBOT_NAVIGATION_MARKER_FOLLOWER_H

#include <memory>

#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <grobot_navigation/exploration/exploration_factory.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class MarkerFollower {
 public:
  MarkerFollower();
  ~MarkerFollower();

 private:
  void initExploration();
  void initFollower();
  void clearCostmapsTimerCallback(const ros::TimerEvent& event);
  void fiducialTransformsCallback(
      const fiducial_msgs::FiducialTransformArrayConstPtr& msg);

  ros::NodeHandle gnh_, pnh_;
  std::string logger_prefix_;

  ros::Timer clear_costmaps_timer_;
  ros::ServiceClient clear_costmaps_service_client_;

  tf::TransformBroadcaster transform_broadcaster_;
  std::shared_ptr<tf::TransformListener> transform_listener_;
  ros::Subscriber transforms_subscriber_;
  ros::Publisher move_base_publisher_;

  int32_t marker_id_;
  tf::Transform marker_to_marker_pose_;
  tf::Transform marker_to_target_pose_;
  ros::Time last_update_time_;
  ros::Duration update_timeout_;

  exploration::ExplorationBasePtr exploration_;
};

#endif  // GROBOT_NAVIGATION_MARKER_FOLLOWER_H
