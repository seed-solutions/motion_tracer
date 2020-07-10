#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <fstream>  //iostream file
#include <ros/package.h>

//////////////////////////////
class JoyRecording{
public:
  JoyRecording(ros::NodeHandle _nh,std::string _file);
  void GetJoy(const sensor_msgs::JoyPtr& _data);

private:
  ros::NodeHandle nh_;
  ros::Subscriber ps3_sub_;
  std::string pkg_path_;

  ros::Time start_time_;
  std::string file_;
  int sec;
};

JoyRecording::JoyRecording(const ros::NodeHandle _nh,std::string _file) :
  nh_(_nh),file_(_file)
{
  ps3_sub_ = nh_.subscribe("/joy",1, &JoyRecording::GetJoy,this);
  pkg_path_ = ros::package::getPath("motion_tracer");
  std::ofstream ofs(pkg_path_ + "/capture/joy_data"+ file_ +".csv",std::ios_base::trunc); //delete file
  start_time_ = ros::Time::now();

  ofs.app; //create data
  //headder same as ps3joy http://wiki.ros.org/ps3joy
  ofs << "time,";
  for(int i=0; i < 20;++i){
    ofs << "axes[" << i << "],";
  }
  for(int i=0; i < 16;++i){
    ofs << "buttons[" << i << "],";
  }
  ofs << "buttons[16]" << std::endl;
}

void JoyRecording::GetJoy(const sensor_msgs::JoyPtr& _data){
  std::ofstream ofs(pkg_path_ + "/capture/joy_data"+ file_ +".csv",std::ios_base::app);  //add data
    ofs << (ros::Time::now() - start_time_).toSec() << ",";
    for(int i=0; i < _data->axes.size() ; ++i){
      ofs << _data->axes[i] << ",";
      std::cout << _data->axes[i] << ",";
    }
    for(int i=0; i < _data->buttons.size()-1 ; ++i){
      ofs << _data->buttons[i] << ",";
      std::cout << _data->buttons[i] << ",";
    }
    ofs << _data->buttons[_data->buttons.size()-1] << std::endl;
    std::cout << _data->buttons[_data->buttons.size()-1] << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"joy_recording_node");
  ros::NodeHandle nh;

  std::string file_name;

  if(argc != 1) file_name=argv[1];
  else file_name="";

  JoyRecording jr(nh,file_name);

  ros::spin();

  return 0;
}
