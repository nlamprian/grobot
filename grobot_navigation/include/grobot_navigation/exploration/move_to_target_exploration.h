#ifndef GROBOT_NAVIGATION_EXPLORATION_MOVE_TO_TARGET_EXPLORATION_H
#define GROBOT_NAVIGATION_EXPLORATION_MOVE_TO_TARGET_EXPLORATION_H

#include <grobot_navigation/exploration/exploration_base.h>
#include <tf/transform_datatypes.h>

namespace exploration {

class MoveToTargetExploration : public ExplorationBase {
 public:
  MoveToTargetExploration()
      : ExplorationBase(Strategy::MOVE_TO_TARGET, "MoveToTargetExploration: ") {
  }

  virtual bool init(ros::NodeHandle& nh) override {
    if (not ExplorationBase::init(nh)) return false;

    // Initialize target pose
    ros::NodeHandle pose_nh(nh, "target_pose");
    double x, y, yaw;
    if (not pose_nh.getParam("x", x)) {
      ROS_ERROR_STREAM(logger_prefix_ << "Failed to get x");
      return false;
    }
    target_pose_.getOrigin().setX(x);
    if (not pose_nh.getParam("y", y)) {
      ROS_ERROR_STREAM(logger_prefix_ << "Failed to get y");
      return false;
    }
    target_pose_.getOrigin().setY(y);
    if (not pose_nh.getParam("yaw", yaw)) {
      ROS_ERROR_STREAM(logger_prefix_ << "Failed to get yaw");
      return false;
    }
    target_pose_.setRotation(tf::createQuaternionFromYaw(yaw));
    if (not pose_nh.getParam("frame_id", target_pose_.frame_id_)) {
      ROS_ERROR_STREAM(logger_prefix_ << "Failed to get frame_id");
      return false;
    }

    return true;
  }

  /**
   * @brief Starts exploring the space by moving to a known target and waiting
   * until the marker appears in the camera field of view.
   * @return True if exploration has started. False, otherwise
   */
  virtual bool start() override {
    if (exploring_) return false;

    // Get current robot pose
    tf::StampedTransform robot_pose;
    try {
      transform_listener_->lookupTransform("map", robot_frame_, ros::Time(0),
                                           robot_pose);
    } catch (const tf::TransformException& error) {
      ROS_WARN_STREAM(logger_prefix_ << error.what());
      return false;
    }

    // Compute distance from target
    tf::Pose error_pose = robot_pose.inverse() * target_pose_;
    if (error_pose.getOrigin().length() < 0.2 and
        tf::getYaw(error_pose.getRotation()) < 0.35)
      return false;

    // Prepare move base goal
    move_base_msgs::MoveBaseGoal goal;
    tf::poseStampedTFToMsg(target_pose_, goal.target_pose);
    goal.target_pose.header.stamp = ros::Time::now();

    move_base_action_client_->sendGoal(
        goal,
        boost::bind(&MoveToTargetExploration::doneCallback, this, _1, _2));

    exploring_ = true;
    ROS_INFO_STREAM(logger_prefix_ << "Started exploration");
    return true;
  }

 private:
  tf::Stamped<tf::Transform> target_pose_;
};

using MoveToTargetExplorationPtr = std::shared_ptr<MoveToTargetExploration>;
using MoveToTargetExplorationConstPtr =
    std::shared_ptr<const MoveToTargetExploration>;

}  // namespace exploration

#endif  // GROBOT_NAVIGATION_EXPLORATION_MOVE_TO_TARGET_EXPLORATION_H
