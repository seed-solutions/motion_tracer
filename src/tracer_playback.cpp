#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <fstream>  //iostream file
#include <ros/package.h>

//for dynamic reconfigure
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

//////////////////////////////
class TracerPlayback{
public:
  TracerPlayback(ros::NodeHandle _nh,std::string _file);
  void Play();

private:
  ros::NodeHandle nh_;
  std::string pkg_path_;
  ros::Time start_time_;

  ros::Publisher tracer_state_pub_;
  sensor_msgs::JointState tracer_state_;
  std::string file_;

  dynamic_reconfigure::ReconfigureRequest srv_req_;
  dynamic_reconfigure::ReconfigureResponse srv_resp_;
  dynamic_reconfigure::BoolParameter enable_param_;
  dynamic_reconfigure::Config conf_;
};

TracerPlayback::TracerPlayback(const ros::NodeHandle _nh,std::string _file) :
  nh_(_nh),file_(_file)
{
  //-------change dynamic reconfigure parameters----------
  enable_param_.name = "publish_joint_state_enable";
  enable_param_.value = false;
  conf_.bools.push_back(enable_param_);

  enable_param_.name = "publish_joy_enable";
  enable_param_.value = false;
  conf_.bools.push_back(enable_param_);
  srv_req_.config = conf_;

  ros::service::call("/tracer_teleop_node/set_parameters", srv_req_, srv_resp_);
  //----------------------------------------------------

  pkg_path_ = ros::package::getPath("motion_tracer");
  tracer_state_pub_ = nh_.advertise<sensor_msgs::JointState>("tracer_states",1);
  start_time_ = ros::Time::now();

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

}

void TracerPlayback::Play(){
  std::ifstream ifs(pkg_path_ + "/capture/tracer_data"+ file_ +".csv",std::ios_base::in);
  std::string headder;
  getline(ifs,headder); //read headder

  std::string str;
  float data[tracer_state_.name.size() + 1] = {0};

  while(getline(ifs,str)){
    std::string token;
    std::istringstream stream(str);
    int i = 0;

    while(getline(stream,token,',')){
      data[i] = std::stof(token);
      i += 1;
    }
    for(int number=0;number<tracer_state_.name.size();++number) tracer_state_.position[number] = data[number+1];
    while((ros::Time::now() - start_time_).toSec() < data[0]);
    tracer_state_.header.stamp = ros::Time::now();
    tracer_state_pub_.publish(tracer_state_);

    for(int i = 1; i < tracer_state_.name.size(); ++i){
      std::cout << data[i] << ",";
    }
    std::cout << std::endl;
  }

  //-------change dynamic reconfigure parameters----------
  conf_.bools.clear();
  enable_param_.name = "publish_joint_state_enable";
  enable_param_.value = true;
  conf_.bools.push_back(enable_param_);

  enable_param_.name = "publish_joy_enable";
  enable_param_.value = true;
  conf_.bools.push_back(enable_param_);
  srv_req_.config = conf_;

  ros::service::call("/tracer_teleop_node/set_parameters", srv_req_, srv_resp_);
  //----------------------------------------------------

  //ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"upper_player_node");
  ros::NodeHandle nh;

  std::string file_name;

  if(argc != 1) file_name=argv[1];
  else file_name="";

  TracerPlayback tp(nh,file_name);
  tp.Play();

  //ros::spin();

  return 0;
}
