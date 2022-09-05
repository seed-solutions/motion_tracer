#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <fstream>  //iostream file
#include <ros/package.h>

//////////////////////////////
class JoyPlayback{
public:
  JoyPlayback(ros::NodeHandle _nh,std::string _file);
  void Play();

private:
  ros::NodeHandle nh_;
  ros::Publisher joy_pub_;
  sensor_msgs::Joy joy_;
  std::string pkg_path_;
  ros::Time start_time_;
  std::string file_;

  int axex_size_ = 6;
  int buttons_size_ = 13;
};

JoyPlayback::JoyPlayback(const ros::NodeHandle _nh,std::string _file) :
  nh_(_nh),file_(_file)
{
  ros::param::set("/tracer/pub_joy_enable",false);

  pkg_path_ = ros::package::getPath("motion_tracer");
  joy_pub_ = nh_.advertise<sensor_msgs::Joy>("/joy",1);

  //same as elecom gamepad
  joy_.axes.resize(axex_size_);
  joy_.buttons.resize(buttons_size_);
  start_time_ = ros::Time::now();
}

void JoyPlayback::Play(){
  std::ifstream ifs(pkg_path_ + "/capture/joy_data"+ file_  +".csv",std::ios_base::in);
  std::string headder;
  getline(ifs,headder); //read headder

  std::string str;
  float data[joy_.axes.size() + joy_.buttons.size() + 1] = {0};

  while(getline(ifs,str)){
    std::string token;
    std::istringstream stream(str);
    int i = 0;

    while(getline(stream,token,',')){
      data[i] = std::stof(token);
      i += 1;
    }
    for(int number=0;number<joy_.axes.size();++number){
      joy_.axes[number] = data[number+1];
    }
    for(int number=0;number<joy_.buttons.size();++number){
      joy_.buttons[number] = data[number+1+joy_.axes.size()];
    }
    while((ros::Time::now() - start_time_).toSec() < data[0]);
    joy_.header.stamp = ros::Time::now();
    joy_pub_.publish(joy_);
  }

  ros::param::set("/tracer/pub_joy_enable",true);
  //ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"joy_playback_node");
  ros::NodeHandle nh;

  std::string file_name;

  if(argc != 1) file_name=argv[1];
  else file_name="";

  JoyPlayback jp(nh,file_name);
  jp.Play();

  //ros::spin();

  return 0;
}
