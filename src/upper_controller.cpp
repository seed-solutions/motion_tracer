#include "motion_tracer/upper_controller.h"

UpperController::UpperController(const ros::NodeHandle _nh) :
 nh_(_nh),r_hand_state("open"),l_hand_state("open")
{
  controller_rate_ = 50;
  controller_cycle_ = (1.0/controller_rate_);
  move_time_ = controller_cycle_;

  tracer_state_sub_ = nh_.subscribe("/tracer_states",1, &UpperController::tracerStateCallback,this);
  joy_sub_ = nh_.subscribe("/joy",1, &UpperController::getJoy,this);
  grasp_client_ =  nh_.serviceClient<seed_r7_ros_controller::HandControl>("seed_r7_ros_controller/hand_control");

  init_follow_joint_trajectory();

  neck_movement_ = "increment";
  neck_offset_ = 0;
  neck_reverse_ = false;
  neck_movement_ = "increment";
  neck_auto_ = false;
  neck_reverse_ = false;
  neck_offset_ = 0;
}

void UpperController::init_follow_joint_trajectory()
{
  rarm_client_ = new TrajectoryClient("rarm_controller/follow_joint_trajectory", true);
  larm_client_ = new TrajectoryClient("larm_controller/follow_joint_trajectory", true);
  waist_client_ = new TrajectoryClient("waist_controller/follow_joint_trajectory", true);
  head_client_ = new TrajectoryClient("head_controller/follow_joint_trajectory", true);

  rhand_client_ = new TrajectoryClient("rhand_controller/follow_joint_trajectory", true);
  lhand_client_ = new TrajectoryClient("lhand_controller/follow_joint_trajectory", true);


  while(!rarm_client_->waitForServer(ros::Duration(0.1)) &&
      !larm_client_->waitForServer(ros::Duration(0.1)) &&
      !waist_client_->waitForServer(ros::Duration(0.1)) &&
      !head_client_->waitForServer(ros::Duration(0.1)))
  {
      ROS_INFO("Waiting for the joint_trajectory_action server");
  }

  while(!rhand_client_->waitForServer(ros::Duration(0.1)) && !lhand_client_->waitForServer(ros::Duration(5.0)))
  {
      ROS_INFO("Waiting for the hand joint_trajectory_action server");
  }

  rarm_goal_.trajectory.joint_names.resize(7);
  rarm_goal_.trajectory.joint_names[0] = "r_elbow_joint";
  rarm_goal_.trajectory.joint_names[1] = "r_shoulder_p_joint";
  rarm_goal_.trajectory.joint_names[2] = "r_shoulder_r_joint";
  rarm_goal_.trajectory.joint_names[3] = "r_shoulder_y_joint";
  rarm_goal_.trajectory.joint_names[4] = "r_wrist_p_joint";
  rarm_goal_.trajectory.joint_names[5] = "r_wrist_r_joint";
  rarm_goal_.trajectory.joint_names[6] = "r_wrist_y_joint";
  rarm_goal_.trajectory.points.resize(1);
  rarm_goal_.trajectory.points[0].positions.resize(rarm_goal_.trajectory.joint_names.size());

  larm_goal_.trajectory.joint_names.resize(7);
  larm_goal_.trajectory.joint_names[0] = "l_elbow_joint";
  larm_goal_.trajectory.joint_names[1] = "l_shoulder_p_joint";
  larm_goal_.trajectory.joint_names[2] = "l_shoulder_r_joint";
  larm_goal_.trajectory.joint_names[3] = "l_shoulder_y_joint";
  larm_goal_.trajectory.joint_names[4] = "l_wrist_p_joint";
  larm_goal_.trajectory.joint_names[5] = "l_wrist_r_joint";
  larm_goal_.trajectory.joint_names[6] = "l_wrist_y_joint";
  larm_goal_.trajectory.points.resize(1);
  larm_goal_.trajectory.points[0].positions.resize(larm_goal_.trajectory.joint_names.size());
 
  waist_goal_.trajectory.joint_names.resize(3);
  waist_goal_.trajectory.joint_names[0] = "waist_p_joint";
  waist_goal_.trajectory.joint_names[1] = "waist_r_joint";
  waist_goal_.trajectory.joint_names[2] = "waist_y_joint";
  waist_goal_.trajectory.points.resize(1);
  waist_goal_.trajectory.points[0].positions.resize(waist_goal_.trajectory.joint_names.size());

  head_goal_.trajectory.joint_names.resize(3);
  head_goal_.trajectory.joint_names[0] = "neck_p_joint";
  head_goal_.trajectory.joint_names[1] = "neck_r_joint";
  head_goal_.trajectory.joint_names[2] = "neck_y_joint";
  head_goal_.trajectory.points.resize(1);
  head_goal_.trajectory.points[0].positions.resize(head_goal_.trajectory.joint_names.size());

  rhand_goal_.trajectory.joint_names.resize(1);
  rhand_goal_.trajectory.joint_names[0] = "r_thumb_joint";
  rhand_goal_.trajectory.points.resize(1);
  rhand_goal_.trajectory.points[0].positions.resize(rhand_goal_.trajectory.joint_names.size());

  lhand_goal_.trajectory.joint_names.resize(1);
  lhand_goal_.trajectory.joint_names[0] = "l_thumb_joint";
  lhand_goal_.trajectory.points.resize(1);
  lhand_goal_.trajectory.points[0].positions.resize(lhand_goal_.trajectory.joint_names.size());
}

void UpperController::graspControl(std::string _position, std::string _pose)
{
  if(_position == "right") grasp_srv_.request.position = 0;
  else if(_position == "left")grasp_srv_.request.position = 1;

  grasp_srv_.request.script = _pose;
  grasp_srv_.request.current = 100;

  grasp_client_.call(grasp_srv_);

  if(_position == "right" && _pose == "grasp") r_hand_state = "close";
  else if(_position == "right" && _pose == "release") r_hand_state = "open";

  if(_position == "left" && _pose == "grasp") l_hand_state = "close";
  else if(_position == "left" && _pose == "release") l_hand_state = "open";
}


void UpperController::tracerStateCallback(const sensor_msgs::JointState& _tracer_data)
{
  joint_angles_["waist_y"] = _tracer_data.position[0] / 10.0  * deg2Rad; //waist_y

  joint_angles_["r_shoulder_p"] = _tracer_data.position[1] / 10.0 * deg2Rad * -1.0; //r_shoulder_p
  joint_angles_["r_shoulder_r"] = _tracer_data.position[2] / 10.0 * deg2Rad * -1.0; //r_shoulder_r
  joint_angles_["r_shoulder_y"] = _tracer_data.position[3] / 10.0 * deg2Rad * -1.0; //r_shoulder_y
  joint_angles_["r_elbow"] = (180 - _tracer_data.position[4] / 10.0) * deg2Rad * -1.0; //r_elbow_p
  joint_angles_["r_wrist_y"] = _tracer_data.position[5] / 10.0 * deg2Rad * -1.0; //r_wrist_y
  joint_angles_["r_wrist_p"] = _tracer_data.position[6] / 10.0 * deg2Rad * -1.0; //r_wrist_p
  joint_angles_["r_wrist_r"] = _tracer_data.position[7] / 10.0 * deg2Rad * 1.0; //r_wrist_r
  joint_angles_["r_hand"] = (_tracer_data.position[8] / 10.0 * deg2Rad  + 1.0)* 1.0; //r_hand

  joint_angles_["l_shoulder_p"] = _tracer_data.position[9] / 10.0 * deg2Rad * -1.0; //l_shoulder_p
  joint_angles_["l_shoulder_r"] = _tracer_data.position[10] / 10.0 * deg2Rad * 1.0; //l_shoulder_r
  joint_angles_["l_shoulder_y"] = _tracer_data.position[11] / 10.0 * deg2Rad * 1.0; //l_shoulder_y
  joint_angles_["l_elbow"] = (180- _tracer_data.position[12] / 10.0) * deg2Rad * -1.0; //l_elbow_p
  joint_angles_["l_wrist_y"] = _tracer_data.position[13] / 10.0 * deg2Rad * 1.0; //l_wrist_y
  joint_angles_["l_wrist_p"] = _tracer_data.position[14] / 10.0 * deg2Rad * -1.0; //l_wrist_p
  joint_angles_["l_wrist_r"] = _tracer_data.position[15] / 10.0 * deg2Rad * -1.0; //l_wrist_r
  joint_angles_["l_hand"] = (_tracer_data.position[16] / 10.0 * deg2Rad + 0.8) * -1.0; //l_hand

//  joint_angles_["waist_p"] = 0; //waist_p
//  joint_angles_["waist_r"] = 0; //waist_r
//  joint_angles_["neck_y"] = 0; //neck_y
//  joint_angles_["neck_r"] = 0; //neck_r
//  joint_angles_["neck_p"] = 0; //neck_p

  sendJointAngles();
}

void UpperController::sendJointAngles()
{
  rarm_goal_.trajectory.points[0].positions[0] = joint_angles_["r_elbow"];
  rarm_goal_.trajectory.points[0].positions[1] = joint_angles_["r_shoulder_p"];
  rarm_goal_.trajectory.points[0].positions[2] = joint_angles_["r_shoulder_r"];
  rarm_goal_.trajectory.points[0].positions[3] = joint_angles_["r_shoulder_y"];
  rarm_goal_.trajectory.points[0].positions[4] = joint_angles_["r_wrist_p"];
  rarm_goal_.trajectory.points[0].positions[5] = joint_angles_["r_wrist_r"];
  rarm_goal_.trajectory.points[0].positions[6] = joint_angles_["r_wrist_y"];
  rarm_goal_.trajectory.points[0].time_from_start = ros::Duration(move_time_);

  larm_goal_.trajectory.points[0].positions[0] = joint_angles_["l_elbow"];
  larm_goal_.trajectory.points[0].positions[1] = joint_angles_["l_shoulder_p"];
  larm_goal_.trajectory.points[0].positions[2] = joint_angles_["l_shoulder_r"];
  larm_goal_.trajectory.points[0].positions[3] = joint_angles_["l_shoulder_y"];
  larm_goal_.trajectory.points[0].positions[4] = joint_angles_["l_wrist_p"];
  larm_goal_.trajectory.points[0].positions[5] = joint_angles_["l_wrist_r"];
  larm_goal_.trajectory.points[0].positions[6] = joint_angles_["l_wrist_y"];
  larm_goal_.trajectory.points[0].time_from_start = ros::Duration(move_time_);

  waist_goal_.trajectory.points[0].positions[0] = joint_angles_["waist_p"];
  waist_goal_.trajectory.points[0].positions[1] = joint_angles_["waist_r"];
  waist_goal_.trajectory.points[0].positions[2] = joint_angles_["waist_y"];
  waist_goal_.trajectory.points[0].time_from_start = ros::Duration(move_time_);

  head_goal_.trajectory.points[0].positions[0] = joint_angles_["neck_p"];
  head_goal_.trajectory.points[0].positions[1] = joint_angles_["neck_r"];
  head_goal_.trajectory.points[0].positions[2] = joint_angles_["neck_y"];
  head_goal_.trajectory.points[0].time_from_start = ros::Duration(move_time_);

  rhand_goal_.trajectory.points[0].positions[0] = joint_angles_["r_hand"];
  rhand_goal_.trajectory.points[0].time_from_start = ros::Duration(move_time_);

  lhand_goal_.trajectory.points[0].positions[0] = joint_angles_["l_hand"];
  lhand_goal_.trajectory.points[0].time_from_start = ros::Duration(move_time_);

  rarm_goal_.trajectory.header.stamp = 
      larm_goal_.trajectory.header.stamp =
      waist_goal_.trajectory.header.stamp =
      head_goal_.trajectory.header.stamp = ros::Time::now() + ros::Duration(controller_cycle_);

  rhand_goal_.trajectory.header.stamp = 
      lhand_goal_.trajectory.header.stamp = ros::Time::now() + ros::Duration(controller_cycle_);

  rarm_client_->sendGoal(rarm_goal_);
  larm_client_->sendGoal(larm_goal_);
  waist_client_->sendGoal(waist_goal_);
  head_client_->sendGoal(head_goal_);

//------------  hand control  -----------------------------------
  //rhand_client_->sendGoal(rhand_goal_);
  //lhand_client_->sendGoal(lhand_goal_);

  if(joint_angles_["r_hand"] < 0.0 && r_hand_state == "open" ){
    graspControl("right","grasp");
  }
  else if (joint_angles_["r_hand"] >= 0.0 && r_hand_state == "close" ){
    graspControl("right","release");
  }
  if(joint_angles_["l_hand"] > 0.0 && l_hand_state == "open" ){
    graspControl("left","grasp");
  }
  else if (joint_angles_["l_hand"] <= 0.0 && l_hand_state == "close" ){
    graspControl("left","release");
  }
//------------------------------------------------------------

}

void UpperController::getJoy(const sensor_msgs::JoyPtr& _ps3){
  double look_right = 0;
  double look_left = 0;

  //If you don't want to look at hand, you should comment out below.
  look_right = -joint_angles_["r_shoulder_r"] - joint_angles_["r_shoulder_y"];
  look_left = joint_angles_["l_shoulder_r"] + joint_angles_["l_shoulder_y"];

  //std::cout << joint_angles_["r_shoulder_r"] << "," <<  joint_angles_["r_shoulder_y"] << std::endl;

  int sign = 1;
  if(neck_reverse_) sign = -1;
  else sign = 1;

  // for waist pitch & roll
  if(_ps3->buttons[7] == 0 && _ps3->buttons[13] == 1 && (std::abs(_ps3->axes[0]) > 0.05 || std::abs(_ps3->axes[1]) > 0.05)) {
    if(neck_movement_ == "absolute"){
      joint_angles_["waist_r"] = (_ps3->axes[0] * 1) ;
      joint_angles_["waist_p"] = sign * (_ps3->axes[1] * -1) ;
    }
    else if(neck_movement_ == "increment"){
      joint_angles_["waist_r"] -= sign * (_ps3->axes[0] * 0.005);
      joint_angles_["waist_p"] -= sign * (_ps3->axes[1] * 0.01);
    }
  }
  else{
    if(neck_movement_ == "absolute"){
      joint_angles_["waist_r"] = 0;
      joint_angles_["waist_p"] = 0;
    }
  }

  //for neck pitch & roll
  if(_ps3->buttons[7] == 0 &&_ps3->buttons[13] == 1 &&  (std::abs(_ps3->axes[2]) > 0.05 || std::abs(_ps3->axes[3]) > 0.05)) {
    if(neck_movement_ == "absolute"){
      joint_angles_["neck_r"] = (_ps3->axes[2] * 2);
      joint_angles_["neck_p"] = neck_offset_*(M_PI/180) + sign * (_ps3->axes[3] * -2);
    }
    else if(neck_movement_ == "increment"){
      joint_angles_["neck_r"] += (_ps3->axes[2] * 0.05);
      joint_angles_["neck_p"] -= sign * (_ps3->axes[3] * 0.05);
    }
  }
  else{ //in case of automatically moving neck  or not
    //std::cout << neck_auto_  << "," << look_right << "," << look_left << std::endl;
    if((neck_auto_ == true) && (look_right > look_left) && (look_right > 0.5)){
      //std::cout << "Right Look" << std::endl;
      joint_angles_["neck_y"] = joint_angles_["r_shoulder_y"];   //assign r_shoulder_y
      if(joint_angles_["r_shoulder_r"] > -0.79) joint_angles_["neck_p"] = (2.36 + joint_angles_["r_elbow"])/2; //assign r_elbow when r_shoulder_r is low
      else joint_angles_["neck_p"] = (1.05 + joint_angles_["r_shoulder_r"]); //assign r_shoulder_r when r_shoulder_r is high
    }
    else if((neck_auto_ == true) && (look_right < look_left) && (look_left > 0.5)) {
      //std::cout << "Left Look" << std::endl;
      joint_angles_["neck_y"] = joint_angles_["l_shoulder_y"];  //assign l_shoulder_y
      if(joint_angles_["l_shoulder_r"] < 0.79) joint_angles_["neck_p"] = (2.36 + joint_angles_["l_elbow"])/2; //assign l_elbow when l_shoulder_r is low
      else joint_angles_["neck_p"] = (1.05 - joint_angles_["l_shoulder_r"]); //assign l_shoulder_r when l_shoulder_r is high
    }
    else if (neck_movement_ == "absolute"){
      joint_angles_["neck_y"] = 0;
      joint_angles_["neck_p"] = neck_offset_*(M_PI/180);
    }
  }

  // set angle limit in joy stick data
  if(joint_angles_["waist_r"] > waist_r_upper_limt) joint_angles_["waist_r"] = waist_r_upper_limt;
  else if(joint_angles_["waist_r"] < waist_r_lower_limt) joint_angles_["waist_r"] = waist_r_lower_limt;
  if(joint_angles_["waist_p"] > waist_p_upper_limt) joint_angles_["waist_p"] = waist_p_upper_limt;
  else if(joint_angles_["waist_p"] < waist_p_lower_limt) joint_angles_["waist_p"] = waist_p_lower_limt;

  if(joint_angles_["neck_r"] > neck_r_upper_limt) joint_angles_["neck_r"] = neck_r_upper_limt;
  else if(joint_angles_["neck_r"] < neck_r_lower_limt) joint_angles_["neck_r"] = neck_r_lower_limt;
  if(joint_angles_["neck_y"] > neck_y_upper_limt) joint_angles_["neck_y"] = neck_y_upper_limt;
  else if(joint_angles_["neck_y"] < neck_y_lower_limt) joint_angles_["neck_y"] = neck_y_lower_limt;
  if(joint_angles_["neck_p"] > neck_p_upper_limt)  joint_angles_["neck_p"] = neck_p_upper_limt;
  else if(joint_angles_["neck_p"] < neck_p_lower_limt)  joint_angles_["neck_p"] = neck_p_lower_limt;

  // set initial pose origin or MC
  if(_ps3->buttons[5] == 1 || _ps3->buttons[15] == 1){
    if(_ps3->buttons[5] != _ps3->buttons[15]){
      joint_angles_["waist_r"] = 0;
      joint_angles_["waist_p"] = 0;
      joint_angles_["neck_y"] = 0;
      joint_angles_["neck_r"] = 0;
      joint_angles_["neck_p"] = neck_offset_*(M_PI/180);
    }
    else if(_ps3->buttons[5] == _ps3->buttons[15]){
       joint_angles_["waist_r"] = 0;
      joint_angles_["waist_p"] = -0.09;
      joint_angles_["neck_y"] = 0;
      joint_angles_["neck_r"] = 0;
      joint_angles_["neck_p"] = 0.17;
    }

  if(_ps3->buttons[13] == 0) sendJointAngles();
  }
}

void callback(motion_tracer::RobotConfig& config, uint32_t level)
{

  ROS_INFO("Reconfigure Request: %s,%d,%d,%d", config.movement.c_str(),
    config.automatically, config.reverse, config.offset);

  neck_movement_ = config.movement.c_str();
  neck_auto_ = config.automatically;
  neck_reverse_ = config.reverse;
  neck_offset_ = config.offset;

}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"upper_controller_node");
  ros::NodeHandle nh;

  UpperController uc(nh);

  dynamic_reconfigure::Server<motion_tracer::RobotConfig> server;
  dynamic_reconfigure::Server<motion_tracer::RobotConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::Rate loop_rate(50);
  while (ros::ok()){
    ros::spinOnce();
    //loop_rate.sleep();
  }
  return 0;
}
