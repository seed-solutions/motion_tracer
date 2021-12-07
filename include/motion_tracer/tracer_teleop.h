#ifndef TRACER_TELEOP_H_
#define TRACER_TELEOP_H_

#include <ros/ros.h>
#include "motion_tracer/tracer_command.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <fstream>  //iostream file

//Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <motion_tracer/TracerConfig.h>

// GUI parameters
bool pub_joint_enable_;
bool pub_joy_enable_;
std::string initial_pose_;

//////////////////////////////
class TracerTeleop{
public:
  TracerTeleop(ros::NodeHandle _nh);
  void get_tracer_data();

private:
  tracer::controller::TracerCommand *tracer_;
  std::vector<uint8_t> tracer_data_;
  std::vector<int16_t> position_;

  ros::Publisher tracer_state_pub_;
  sensor_msgs::JointState tracer_state_;
  ros::NodeHandle nh_;
  ros::Publisher joy_pub_;
  sensor_msgs::Joy joy_;

  bool record_flag_;
  std::string record_filename_;

  //dummy cmd_vel
  ros::Publisher cmd_vel_pub_;
  geometry_msgs::Twist cmd_vel_;

  bool wheel_stop_flag_;

  bool tracer_mode_;
  int init_counter_;

  int16_t pre_r_wrist_y_,pre_l_wrist_y_;

  bool enable_joy_;  // to change 50hz joy pub

};
#endif
