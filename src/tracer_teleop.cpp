#include "motion_tracer/tracer_teleop.h"

TracerTeleop::TracerTeleop(const ros::NodeHandle _nh) :
  nh_(_nh),init_counter_(0),pre_r_wrist_y_(0),pre_l_wrist_y_(0),tracer_mode_(0)
{
  tracer_ = new tracer::controller::TracerCommand();

  if(tracer_->port_open("/dev/tracer_usb",460800)){
  }
  else{
    ROS_ERROR("Connection failed");
    ros::shutdown();
  }

  tracer_state_pub_ = nh_.advertise<sensor_msgs::JointState>("tracer_states",1);
  joy_pub_ = nh_.advertise<sensor_msgs::Joy>("/joy",1);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);

  tracer_state_.name.resize(17);
  tracer_state_.position.resize(17);

  tracer_state_.header.frame_id ="tracer_base";
  tracer_state_.name[0] = "waist_y";
  tracer_state_.name[1] = "r_shoulder_p";
  tracer_state_.name[2] = "r_shoulder_r";
  tracer_state_.name[3] = "r_shoulder_y";
  tracer_state_.name[4] = "r_elbow_p";
  tracer_state_.name[5] = "r_wrist_y";
  tracer_state_.name[6] = "r_wrist_p";
  tracer_state_.name[7] = "r_wrist_r";
  tracer_state_.name[8] = "r_hand";
  tracer_state_.name[9] = "l_shoulder_p";
  tracer_state_.name[10] = "l_shoulder_r";
  tracer_state_.name[11] = "l_shoulder_y";
  tracer_state_.name[12] = "l_elbow_p";
  tracer_state_.name[13] = "l_wrist_y";
  tracer_state_.name[14] = "l_wrist_p";
  tracer_state_.name[15] = "l_wrist_r";
  tracer_state_.name[16] = "l_hand";

  //same as ps3joy http://wiki.ros.org/ps3joy
  joy_.axes.resize(20);
  fill(joy_.axes.begin(),joy_.axes.end(),0);
  joy_.buttons.resize(17);
  fill(joy_.buttons.begin(),joy_.buttons.end(),0);

  tracer_data_.resize(68);
  fill(tracer_data_.begin(),tracer_data_.end(),0);
  position_.resize(30);
  fill(position_.begin(),position_.end(),0);

  wheel_stop_flag_ = true;
  pub_joint_enable_ = true;
  pub_joy_enable_ = true;
  initial_pose_ = "none";

}

void TracerTeleop::get_tracer_data()
{
  tracer_data_.resize(4);
  fill(tracer_data_.begin(),tracer_data_.end(),0);
  tracer_data_ = tracer_->get_tracer_data(4);

  if(tracer_data_[0] == 0xFD && tracer_data_[1] == 0xDF && tracer_data_[2] == 64 && tracer_data_[3] == 0x14){
    tracer_data_.resize(64);
    fill(tracer_data_.begin(),tracer_data_.end(),0);
    tracer_data_ = tracer_->get_tracer_data(64);
    for(int i=0;i<30;++i) position_[i] = (tracer_data_[i*2+1] << 8) + tracer_data_[i*2+2];
    //waist
    if(-900 <= position_[15] && position_[15] <= 900 )  tracer_state_.position[0] = position_[15] * 5;
    //right arm
    if(-200 <= position_[3] && position_[3] <= 890 )    tracer_state_.position[1] = position_[3];
    else if(-450 < position_[3] && position_[3] < -200 ) tracer_state_.position[1] = -200;
    if(0 <= position_[4] && position_[4] <= 900 )       tracer_state_.position[2] = position_[4];
    if(-900 <= position_[5] && position_[5] <= 900 )    tracer_state_.position[3] = position_[5];
    if(0 <= position_[6] && position_[6] <= 1800 )      tracer_state_.position[4] = position_[6];
    if(-900 <= position_[7] && position_[7] <= 900 ){
      if(pre_r_wrist_y_ > 450 && position_[7] < 0)      tracer_state_.position[5] = 900;
      else if(pre_r_wrist_y_ < -450 && position_[7] > 0)tracer_state_.position[5] = -900;
      else tracer_state_.position[5] = position_[7];
    }
    if(-100 <= position_[8] && position_[8] <= 100 )    tracer_state_.position[6] = position_[8];
    if(-300 <= position_[9] && position_[9] <= 900 )    tracer_state_.position[7] = position_[9];
    tracer_state_.position[8] = position_[11];
    //left arm
    if(-200 <= position_[18] && position_[18] <= 890 )    tracer_state_.position[9] = position_[18];
    else if(-450 < position_[18] && position_[18] < -200 )  tracer_state_.position[9] = -200;
    if(0 <= position_[19] && position_[19] <= 900 )       tracer_state_.position[10] = position_[19];
    if(-900 <= position_[20] && position_[20] <= 900 )    tracer_state_.position[11] = position_[20];
    if(0 <= position_[21] && position_[21] <= 1800 )      tracer_state_.position[12] = position_[21];
    if(-900 <= position_[22] && position_[22] <= 900 ){
      if(pre_l_wrist_y_ > 450 && position_[22] < 0)       tracer_state_.position[13] = 900;
      else if(pre_l_wrist_y_ < -450 && position_[22] > 0) tracer_state_.position[13] = -900;
      else tracer_state_.position[13] = position_[22];
    }
    if(-100 <= position_[23] && position_[23] <= 100 )    tracer_state_.position[14] = position_[23];
    if(-300 <= position_[24] && position_[24] <= 900 )    tracer_state_.position[15] = position_[24];
    tracer_state_.position[16] = position_[26];

    tracer_state_.header.stamp = ros::Time::now();
    if(pub_joint_enable_) tracer_state_pub_.publish(tracer_state_);

    //hand yaw pre position
    pre_r_wrist_y_ = tracer_state_.position[5];
    pre_l_wrist_y_ = tracer_state_.position[13];
  }
  else if(tracer_data_[0] == 0xFB && tracer_data_[1] == 0xBF && tracer_data_[2] == 0x07 && tracer_data_[3] == 0x00){
    tracer_data_.resize(7);
    fill(tracer_data_.begin(),tracer_data_.end(),0);
    tracer_data_ = tracer_->get_tracer_data(7);

    //left joy_stick
    if(tracer_data_[2] > 122 && tracer_data_[2] < 132 ) tracer_data_[2] = 127;
    if(tracer_data_[3] > 122 && tracer_data_[3] < 132 ) tracer_data_[3] = 127;
    if(tracer_data_[4] > 122 && tracer_data_[4] < 132 ) tracer_data_[4] = 127;
    if(tracer_data_[5] > 122 && tracer_data_[5] < 132 ) tracer_data_[5] = 127;
    joy_.axes[0] = static_cast<float>(127 - tracer_data_[2]) / 127;
    joy_.axes[1] = static_cast<float>(127 - tracer_data_[3]) / 127;
    //right joy_stick
    joy_.axes[2] = static_cast<float>(127 - tracer_data_[4]) / 127;
    joy_.axes[3] = static_cast<float>(127 - tracer_data_[5]) / 127;

    //Left Hand
    switch(tracer_data_[0]){
      case 32:
        joy_.buttons[4] = 0;
        joy_.buttons[5] = 1;
        joy_.buttons[6] = 0;
        joy_.buttons[7] = 0;
        break;
      case 128:
        joy_.buttons[4] = 0;
        joy_.buttons[5] = 0;
        joy_.buttons[6] = 0;
        joy_.buttons[7] = 1;
        break;
      case 160:
        joy_.buttons[4] = 0;
        joy_.buttons[5] = 1;
        joy_.buttons[6] = 0;
        joy_.buttons[7] = 1;
        break;
      default:
        joy_.buttons[4] = 0;
        joy_.buttons[5] = 0;
        joy_.buttons[6] = 0;
        joy_.buttons[7] = 0;
    }
    //Right Hand
    switch(tracer_data_[1]){
      case 32:
        joy_.buttons[12] = 0;
        joy_.buttons[13] = 1;
        joy_.buttons[14] = 0;
        joy_.buttons[15] = 0;
        break;
      case 128:
        joy_.buttons[12] = 0;
        joy_.buttons[13] = 0;
        joy_.buttons[14] = 0;
        joy_.buttons[15] = 1;
        break;
      case 160:
        joy_.buttons[12] = 0;
        joy_.buttons[13] = 1;
        joy_.buttons[14] = 0;
        joy_.buttons[15] = 1;
        break;
      default:
        joy_.buttons[12] = 0;
        joy_.buttons[13] = 0;
        joy_.buttons[14] = 0;
        joy_.buttons[15] = 0;
    }

    if((std::abs(joy_.axes[0]) > 0.05 || std::abs(joy_.axes[1]) > 0.05 || std::abs(joy_.axes[2]) > 0.05 || std::abs(joy_.axes[3]) > 0.05) && joy_.buttons[7] == 1) {
      joy_.buttons[8] = 1;
      wheel_stop_flag_ = false;
    }
    else {
      joy_.buttons[8] = 0;
    }
    if(init_counter_ > 0){  //for initial pose
      joy_.buttons[5] = 1;
      joy_.buttons[15] = 1;
      init_counter_ -=1;
    }
    //ros::param::get("/tracer/pub_joy_enable",pub_joy_enable_);
    if(pub_joy_enable_) joy_pub_.publish(joy_);
    //joy_pub_.publish(joy_);

    //dummy cmd_vel publish
    if(joy_.buttons[8] == 0 && wheel_stop_flag_ == false && pub_joy_enable_==true){
      cmd_vel_.linear.x = 0;
      cmd_vel_.linear.y = 0;
      cmd_vel_.linear.z = 0;
      cmd_vel_.angular.x = 0;
      cmd_vel_.angular.y = 0;
      cmd_vel_.angular.z= 0;
      cmd_vel_pub_.publish(cmd_vel_);
      wheel_stop_flag_ = true;
    }

    if(tracer_mode_ == 1 && joy_.buttons[13] == 0 ){
      if(initial_pose_ == "MC"){
        init_counter_ = 50;
        tracer_state_.position[0] = 0;
        //right arm
        tracer_state_.position[1] = -200;
        tracer_state_.position[2] = 100;
        tracer_state_.position[3] = -135;
        tracer_state_.position[4] = 800;
        tracer_state_.position[5] = 0;
        tracer_state_.position[6] = 100;
        tracer_state_.position[7] = 150;
        tracer_state_.position[8] = 1700;

        //left arm
        tracer_state_.position[9] = -200;
        tracer_state_.position[10] = 100;
        tracer_state_.position[11] = -135;
        tracer_state_.position[12] = 800;
        tracer_state_.position[13] = 0;
        tracer_state_.position[14] = 100;
        tracer_state_.position[15] = 150;
        tracer_state_.position[16] = 1700;
        tracer_state_.header.stamp = ros::Time::now();
        tracer_state_pub_.publish(tracer_state_);
        std::cout << "go to initial pose" << std::endl;
      }
      else if(initial_pose_ == "zero"){
        init_counter_ = 20;
        tracer_state_.position[0] = 0;
        //right arm
        tracer_state_.position[1] = 0;
        tracer_state_.position[2] = 0;
        tracer_state_.position[3] = 0;
        tracer_state_.position[4] = 0;
        tracer_state_.position[5] = 0;
        tracer_state_.position[6] = 0;
        tracer_state_.position[7] = 0;
        tracer_state_.position[8] = 300;

        //left arm
        tracer_state_.position[9] = 0;
        tracer_state_.position[10] = 0;
        tracer_state_.position[11] = 0;
        tracer_state_.position[12] = 0;
        tracer_state_.position[13] = 0;
        tracer_state_.position[14] = 0;
        tracer_state_.position[15] = 0;
        tracer_state_.position[16] = 1000;
        tracer_state_.header.stamp = ros::Time::now();
        tracer_state_pub_.publish(tracer_state_);
        std::cout << "go to initial pose" << std::endl;
      }
    }
    tracer_mode_ = joy_.buttons[13];
  }
}

void callback(motion_tracer::TracerConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: %d %d,%s", config.publish_joint_state_enable, 
    config.publish_joy_enable, config.initial_pose.c_str());

  pub_joint_enable_ = config.publish_joint_state_enable;
  pub_joy_enable_ = config.publish_joy_enable;
  initial_pose_ = config.initial_pose.c_str();
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
	ros::init(argc,argv,"tracer_teleop_node");
	ros::NodeHandle nh;

  TracerTeleop tt(nh);

  dynamic_reconfigure::Server<motion_tracer::TracerConfig> server;
  dynamic_reconfigure::Server<motion_tracer::TracerConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  while (ros::ok())
  {
    tt.get_tracer_data();
    ros::spinOnce();
  }
	return 0;
}
