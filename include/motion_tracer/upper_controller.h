#ifndef UPPER_CONTROLLER_H_
#define UPPER_CONTROLLER_H_

#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>

//for status view
#include <std_msgs/String.h>
#include <diagnostic_updater/diagnostic_updater.h>

//for follow joint trajectory
//#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

//for hand controller
#include "seed_r7_ros_controller/HandControl.h"

//Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <motion_tracer/RobotConfig.h>


typedef   actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

// GUI parameters
std::string neck_movement_;
bool neck_auto_;
bool neck_reverse_;
int neck_offset_;

//////////////////////////////
class UpperController{
public:
  UpperController(ros::NodeHandle _nh);
  void init_follow_joint_trajectory();
  void graspControl(std::string _position, std::string _pose);
  void tracerStateCallback(const sensor_msgs::JointState& _tracer_data);
  void sendJointAngles();
  void getJoy(const sensor_msgs::JoyPtr& _ps3);

private:
  ros::NodeHandle nh_;
  ros::Subscriber tracer_state_sub_;
  ros::Subscriber joy_sub_;

  TrajectoryClient* rarm_client_;
  TrajectoryClient* larm_client_;
  TrajectoryClient* waist_client_;
  TrajectoryClient* head_client_;
  TrajectoryClient* rhand_client_;
  TrajectoryClient* lhand_client_;
  control_msgs::FollowJointTrajectoryGoal rarm_goal_,larm_goal_,waist_goal_,head_goal_;
  control_msgs::FollowJointTrajectoryGoal rhand_goal_,lhand_goal_;

  std::map<std::string, double> joint_angles_;

  int controller_rate_; //[Hz]
  double controller_cycle_;  //[sec]
  double move_time_;  //[sec]

  float rad2Deg = 180.0 / M_PI;
  float deg2Rad = M_PI / 180.0;

  std::string r_hand_state, l_hand_state;

  // angle limit
  const float neck_p_upper_limt = 1.0;
  const float neck_p_lower_limt = -0.3;
  const float neck_r_upper_limt = 0.087;
  const float neck_r_lower_limt = -0.087;
  const float neck_y_upper_limt = 1.57;
  const float neck_y_lower_limt = -1.57;

  const float waist_p_upper_limt = 0.68;
  const float waist_p_lower_limt = -0.16;
  const float waist_r_upper_limt = 0.12;
  const float waist_r_lower_limt = -0.12;

  //grasp control
  ros::ServiceClient grasp_client_;
  seed_r7_ros_controller::HandControl grasp_srv_;

};

#endif
