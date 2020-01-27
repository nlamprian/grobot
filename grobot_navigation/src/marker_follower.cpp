#include <algorithm>
#include <cmath>

#include <grobot_navigation/marker_follower.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>

MarkerFollower::MarkerFollower()
    : pnh_("~"),
      logger_prefix_("MarkerFollower: "),
      transform_listener_(std::make_shared<tf::TransformListener>()),
      update_timeout_(4) {
  initExploration();
  initFollower();

  move_base_publisher_ =
      gnh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);

  transforms_subscriber_ =
      gnh_.subscribe("fiducial_transforms", 1,
                     &MarkerFollower::fiducialTransformsCallback, this);

  clear_costmaps_service_client_ =
      gnh_.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
  clear_costmaps_timer_ = gnh_.createTimer(
      ros::Duration(5), &MarkerFollower::clearCostmapsTimerCallback, this);

  ROS_INFO_STREAM(logger_prefix_ << "Initialized");
}

MarkerFollower::~MarkerFollower() {}

void MarkerFollower::initExploration() {
  ros::NodeHandle enh(pnh_, "exploration");
  exploration_ = exploration::ExplorationFactory::get(enh);
  exploration_->setTransformListener(transform_listener_);
}

void MarkerFollower::initFollower() {
  // Initialize marker id
  if (not pnh_.getParam("marker_id", marker_id_))
    std::runtime_error("Failed to get marker_id");

  // Initialize marker to marker pose (fixes orientation; z axis is vertical)
  marker_to_marker_pose_.setRotation(
      tf::createQuaternionFromRPY(0, M_PI_2, M_PI_2));

  // Initialize marker to target pose
  double x, y;
  if (not pnh_.getParam("marker_to_target_pose/x", x))
    std::runtime_error("Failed to get marker_to_target_pose/x");
  if (not pnh_.getParam("marker_to_target_pose/y", y))
    std::runtime_error("Failed to get marker_to_target_pose/y");
  marker_to_target_pose_.getOrigin().setX(x);
  marker_to_target_pose_.getOrigin().setY(y);
  marker_to_target_pose_.setRotation(tf::createQuaternionFromYaw(0));
}

void MarkerFollower::clearCostmapsTimerCallback(
    const ros::TimerEvent& /*event*/) {
  std_srvs::Empty srv;
  clear_costmaps_service_client_.call(srv);
}

void MarkerFollower::fiducialTransformsCallback(
    const fiducial_msgs::FiducialTransformArrayConstPtr& msg) {
  // Check if marker is detected
  fiducial_msgs::FiducialTransformArray::_transforms_type::const_iterator it;
  it = std::find_if(msg->transforms.begin(), msg->transforms.end(),
                    [&](const fiducial_msgs::FiducialTransformArray::
                            _transforms_type::value_type& transform) {
                      return transform.fiducial_id == marker_id_;
                    });
  if (it == msg->transforms.end()) {  // Lost track of marker
    if (msg->header.stamp - last_update_time_ > update_timeout_)
      exploration_->start();
    return;
  } else if (exploration_->isExploring()) {  // Tracked marker down
    exploration_->cancel();
    last_update_time_ = msg->header.stamp;
    return;
  }

  // Get camera to marker pose
  tf::Stamped<tf::Transform> marker_pose;
  marker_pose.stamp_ = msg->header.stamp;
  marker_pose.frame_id_ = msg->header.frame_id;
  tf::transformMsgToTF(it->transform, marker_pose);
  // Orient marker frame
  marker_pose *= marker_to_marker_pose_;

  // Check if marker is too close
  tf::Stamped<tf::Transform> robot_pose;
  try {
    transform_listener_->transformPose("base_footprint", marker_pose,
                                       robot_pose);
  } catch (const tf::TransformException& error) {
    ROS_WARN_STREAM(logger_prefix_ << error.what());
    return;
  }
  robot_pose.getOrigin().setZ(0);
  if (0.5 * robot_pose.getOrigin().length() <
      marker_to_target_pose_.getOrigin().length()) {
    last_update_time_ = msg->header.stamp;
    return;
  }

  // Transform marker pose to map frame
  try {
    transform_listener_->transformPose("map", marker_pose, marker_pose);
  } catch (const tf::TransformException& error) {
    ROS_WARN_STREAM(logger_prefix_ << error.what());
    return;
  }
  // Apply marker to target transform
  tf::Stamped<tf::Transform> target_pose(marker_pose);
  target_pose *= marker_to_target_pose_;
  // Clean target pose
  target_pose.getOrigin().setZ(0);
  double yaw = tf::getYaw(target_pose.getRotation());
  target_pose.setRotation(tf::createQuaternionFromYaw(yaw));

  // Send move base goal
  geometry_msgs::PoseStamped target_pose_msg;
  tf::poseStampedTFToMsg(target_pose, target_pose_msg);
  move_base_publisher_.publish(target_pose_msg);

  // Update tf target pose
  tf::StampedTransform target_transform(target_pose, target_pose.stamp_,
                                        target_pose.frame_id_, "target");
  transform_broadcaster_.sendTransform(target_transform);

  last_update_time_ = target_pose.stamp_;
}
