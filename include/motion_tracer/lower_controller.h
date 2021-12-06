#ifndef LOWER_CONTROLLER_H_
#define LOEWR_CONTROLLER_H_

#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <fstream>  //iostream file

//for status view
#include <std_msgs/String.h>
#include <diagnostic_updater/diagnostic_updater.h>

//for follow joint trajectory
//#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

//for led controller
#include "seed_r7_ros_controller/LedControl.h"


//using namespace robot_hardware;

typedef   actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

//////////////////////////////
class LowerController{
public:
  LowerController(ros::NodeHandle _nh);
  void init_follow_joint_trajectory();
  void jointStateCallback(const sensor_msgs::JointState& _joint_data);
  void sendJointAngles();
  void getJoy(const sensor_msgs::JoyPtr& _ps3);
  void diagnosticsCallback(const diagnostic_msgs::DiagnosticArrayPtr& _msg);

private:
  ros::NodeHandle nh_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber diag_sub_;

  TrajectoryClient* lifter_client_;

  control_msgs::FollowJointTrajectoryGoal lifter_goal_;

  std::map<std::string, double> joint_angles_;

  int controller_rate_; //[Hz]
  double controller_cycle_;  //[sec]
  double move_time_;  //[sec]

  float rad2Deg = 180.0 / M_PI;
  float deg2Rad = M_PI / 180.0;

  // angle limit
  const float ankle_upper_limt = 1.57;
  const float ankle_lower_limt = 0;
  const float knee_upper_limt = 0;
  const float knee_lower_limt = -1.57;

  float lifter_ratio_;
  bool init,on_protective_stop;

  //led control
  ros::ServiceClient led_client_;
  seed_r7_ros_controller::LedControl led_srv_;

};

#endif
