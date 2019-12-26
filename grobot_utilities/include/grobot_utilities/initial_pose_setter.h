#ifndef GROBOT_GAZEBO_INITIAL_POSE_SETTER
#define GROBOT_GAZEBO_INITIAL_POSE_SETTER

#include <algorithm>

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

/**
 * \brief Relocalizes a gazebo robot.
 * \details Subscribes to the /gazebo/model_states topic, gets the pose of the
 * requested model, and sends a message to the /initialpose topic, thus
 * relocalizing the robot.
 * \note Use like this:
 * rosrun grobot_utilities set_initial_pose _model_name:=<model-name>
 */
class InitialPoseSetter {
 public:
  InitialPoseSetter() : pnh_("~") {
    bool found = pnh_.getParam("model_name", model_name_);
    if (not found) throw std::runtime_error("Failed to get ~model_name");

    initial_pose_publisher_ =
        gnh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",
                                                                 1);

    model_states_subscriber_ =
        gnh_.subscribe("/gazebo/model_states", 1,
                       &InitialPoseSetter::modelStatesCallback, this);
  }

 private:
  void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr& msg) {
    auto it = std::find(msg->name.begin(), msg->name.end(), model_name_);
    if (it == msg->name.end()) return;

    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.pose = msg->pose[std::distance(msg->name.begin(), it)];
    initial_pose_publisher_.publish(pose_msg);
    ROS_INFO_STREAM("InitialPoseSetter: Set pose");

    ros::shutdown();
  }

  ros::NodeHandle gnh_, pnh_;
  ros::Subscriber model_states_subscriber_;
  ros::Publisher initial_pose_publisher_;

  std::string model_name_;
};

#endif  // GROBOT_GAZEBO_INITIAL_POSE_SETTER
