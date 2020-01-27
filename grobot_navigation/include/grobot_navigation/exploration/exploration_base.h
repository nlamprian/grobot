#ifndef GROBOT_NAVIGATION_EXPLORATION_EXPLORATION_BASE_H
#define GROBOT_NAVIGATION_EXPLORATION_EXPLORATION_BASE_H

#include <atomic>
#include <memory>
#include <string>

#include <actionlib/client/simple_action_client.h>
#include <grobot_navigation/exploration/types.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>

namespace exploration {

/**
 * @brief An abstract interface that can be extended to implement a specific
 * exploration strategy.
 */
class ExplorationBase {
  using MoveBaseClient =
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
  using MoveBaseClientPtr = std::shared_ptr<MoveBaseClient>;

 public:
  ExplorationBase(Strategy strategy, const std::string& logger_prefix)
      : strategy_(strategy),
        logger_prefix_(logger_prefix),
        exploring_(false),
        cancelled_(false) {
    move_base_action_client_ =
        std::make_shared<MoveBaseClient>("/move_base", true);
    move_base_action_client_->waitForServer();
  }

  Strategy getStrategy() const { return strategy_; }

  void setTransformListener(
      const std::shared_ptr<tf::TransformListener>& transform_listener) {
    transform_listener_ = transform_listener;
  }

  /**
   * @brief Initializes the class.
   * @param nh Namespace under which to look for parameters
   * @return True on successful initialization. False, otherwise
   */
  virtual bool init(ros::NodeHandle& nh) {
    // Initialize robot frame
    if (not nh.getParam("robot_frame", robot_frame_)) {
      ROS_ERROR_STREAM(logger_prefix_ << "Failed to get robot_frame");
      return false;
    }
    robot_frame_ = tf::resolve(tf::getPrefixParam(nh), robot_frame_);

    return true;
  }

  /**
   * @brief Start exploration based on the implemented strategy.
   * @return True if conditions are met and exploration has started. False,
   * otherwise
   */
  virtual bool start() = 0;

  /**
   * @brief Checks if an exploration is in progress.
   * @return True if there is an on-going exploration. False, otherwise
   */
  bool isExploring() const { return exploring_; }

  /**
   * @brief Cancels current exploration.
   */
  void cancel() {
    if (cancelled_) return;
    move_base_action_client_->cancelGoal();
    cancelled_ = true;
    ROS_INFO_STREAM(logger_prefix_ << "Cancelled exploration");
  }

 protected:
  virtual void doneCallback(
      const actionlib::SimpleClientGoalState& state,
      const move_base_msgs::MoveBaseResultConstPtr& result) {
    exploring_ = false;
    cancelled_ = false;
    ROS_INFO_STREAM(logger_prefix_ << "Exploration is done");
  }

  const Strategy strategy_;
  const std::string logger_prefix_;

  std::string robot_frame_;

  std::shared_ptr<tf::TransformListener> transform_listener_;

  MoveBaseClientPtr move_base_action_client_;
  std::atomic_bool exploring_;
  std::atomic_bool cancelled_;
};

using ExplorationBasePtr = std::shared_ptr<ExplorationBase>;
using ExplorationBaseConstPtr = std::shared_ptr<const ExplorationBase>;

}  // namespace exploration

#endif  // GROBOT_NAVIGATION_EXPLORATION_EXPLORATION_BASE_H
