#include "motion_tracer/lower_controller.h"

LowerController::LowerController(const ros::NodeHandle _nh) :
  nh_(_nh),lifter_ratio_(0.01),init(true)
{

  controller_rate_ = 50;
  controller_cycle_ = (1.0/controller_rate_);
  move_time_ = 0.5;//controller_cycle_;

  joint_state_sub_ = nh_.subscribe("/joint_states",2, &LowerController::jointStateCallback,this);
  joy_sub_ = nh_.subscribe("/joy",2, &LowerController::getJoy,this);

  init_follow_joint_trajectory();

}

void LowerController::init_follow_joint_trajectory()
{
  lifter_client_ = new TrajectoryClient("lifter_controller/follow_joint_trajectory", true);

  while(!lifter_client_->waitForServer(ros::Duration(1)) )
  {
      ROS_INFO("Waiting for the lifter joint_trajectory_action server");
  }

  lifter_goal_.trajectory.joint_names.resize(2);
  lifter_goal_.trajectory.joint_names[0] = "ankle_joint";
  lifter_goal_.trajectory.joint_names[1] = "knee_joint";
  lifter_goal_.trajectory.points.resize(1);
  lifter_goal_.trajectory.points[0].positions.resize(lifter_goal_.trajectory.joint_names.size());
}

void LowerController::jointStateCallback(const sensor_msgs::JointState& _joint_data)
{
  if(init)
  {
    joint_angles_["ankle"] = _joint_data.position[0];
    joint_angles_["knee"] = _joint_data.position[1];
  }
  init = false;

}
void LowerController::sendJointAngles()
{
  lifter_goal_.trajectory.points[0].positions[0] = joint_angles_["ankle"];
  lifter_goal_.trajectory.points[0].positions[1] = joint_angles_["knee"];
  lifter_goal_.trajectory.points[0].time_from_start = ros::Duration(move_time_);

  lifter_goal_.trajectory.header.stamp = ros::Time::now() + ros::Duration(controller_cycle_);

  lifter_client_->sendGoal(lifter_goal_);

}

void LowerController::getJoy(const sensor_msgs::JoyPtr& _ps3)
{
    if((_ps3->buttons[8] == 1 || _ps3->buttons[10] == 1) && _ps3->axes[3] != 0){
    joint_angles_["ankle"] -= (_ps3->axes[3] * lifter_ratio_);
    joint_angles_["knee"] += (_ps3->axes[3] * lifter_ratio_);
    if(joint_angles_["ankle"] > ankle_upper_limt) joint_angles_["ankle"] = ankle_upper_limt;
    else if(joint_angles_["ankle"] < ankle_lower_limt) joint_angles_["ankle"] = ankle_lower_limt;
    if(joint_angles_["knee"] > knee_upper_limt) joint_angles_["knee"] = knee_upper_limt;
    else if(joint_angles_["knee"] < knee_lower_limt) joint_angles_["knee"] = knee_lower_limt;

    sendJointAngles();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"lower_controller_node");
  ros::NodeHandle nh;

  LowerController lc(nh);

  ros::Rate loop_rate(50);
  while (ros::ok()){
    ros::spinOnce();
    //loop_rate.sleep();
  }

  return 0;
}