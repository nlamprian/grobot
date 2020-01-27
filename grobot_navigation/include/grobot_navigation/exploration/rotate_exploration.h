#ifndef GROBOT_NAVIGATION_EXPLORATION_ROTATE_EXPLORATION_H
#define GROBOT_NAVIGATION_EXPLORATION_ROTATE_EXPLORATION_H

#include <cmath>

#include <grobot_navigation/exploration/exploration_base.h>
#include <tf/transform_datatypes.h>

namespace exploration {

class RotateExploration : public ExplorationBase {
 public:
  RotateExploration()
      : ExplorationBase(Strategy::ROTATE, "RotateExploration: ") {}

  virtual bool init(ros::NodeHandle& nh) override {
    if (not ExplorationBase::init(nh)) return false;

    // Initialize target pose
    target_pose_.frame_id_ = robot_frame_;
    target_pose_.setOrigin(tf::Vector3(0, 0, 0));
    target_pose_.setRotation(tf::createQuaternionFromYaw(2 * M_PI / 3));

    return true;
  }

  /**
   * @brief Starts exploring the space by rotating around the current position
   * until the marker appears in the camera field of view.
   * @return True if exploration has started. False, otherwise
   */
  virtual bool start() override {
    if (exploring_) return false;

    // Get target pose
    auto target_pose = target_pose_;
    target_pose.stamp_ = ros::Time(0);
    try {
      transform_listener_->transformPose("map", target_pose, target_pose);
    } catch (const tf::TransformException& error) {
      ROS_WARN_STREAM(logger_prefix_ << error.what());
      return false;
    }

    // Prepare move base goal
    move_base_msgs::MoveBaseGoal goal;
    tf::poseStampedTFToMsg(target_pose, goal.target_pose);

    move_base_action_client_->sendGoal(
        goal, boost::bind(&RotateExploration::doneCallback, this, _1, _2));

    exploring_ = true;
    ROS_INFO_STREAM(logger_prefix_ << "Started exploration");
    return true;
  }

 private:
  tf::Stamped<tf::Transform> target_pose_;
};

using RotateExplorationPtr = std::shared_ptr<RotateExploration>;
using RotateExplorationConstPtr = std::shared_ptr<const RotateExploration>;

}  // namespace exploration

#endif  // GROBOT_NAVIGATION_EXPLORATION_ROTATE_EXPLORATION_H
