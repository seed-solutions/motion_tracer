#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <fstream>  //iostream file
#include <sensor_msgs/JointState.h>
#include <ros/package.h>

//////////////////////////////
class TracerRecording{
public:
  TracerRecording(ros::NodeHandle _nh,std::string _file);
  void JointStateCallback(const sensor_msgs::JointState& _tracer_data);

private:
  ros::NodeHandle nh_;
  ros::Subscriber tracer_state_sub_;
  std::string pkg_path_;

  ros::Time start_time_;
  std::string file_;
};


TracerRecording::TracerRecording(const ros::NodeHandle _nh,std::string _file) :
  nh_(_nh),file_(_file)
{
  tracer_state_sub_ = nh_.subscribe("/tracer_states",1,&TracerRecording::JointStateCallback,this);
  pkg_path_ = ros::package::getPath("motion_tracer");
  std::ofstream ofs(pkg_path_ + "/capture/tracer_data" + file_ +".csv",std::ios_base::trunc); //delete file
  start_time_ = ros::Time::now();

  ofs.app; //create data
  //headder
  ofs << "time," << "waist_y,";
  ofs << "r_shoulder_p," << "r_shoulder_r," << "r_shoulder_y," << "r_elbow_p," << "r_wrist_y," ;
  ofs << "r_wrist_p," << "r_wrist_r," << "r_hand,";
  ofs << "l_shoulder_p," << "l_shoulder_r," << "l_shoulder_y," << "l_elbow_p," << "l_wrist_y," ;
  ofs << "l_wrist_p," << "l_wrist_r," << "l_hand";
  ofs << std::endl;

}

void TracerRecording::JointStateCallback(const sensor_msgs::JointState& _tracer_data)
  {
    std::ofstream ofs(pkg_path_ + "/capture/tracer_data" + file_ +".csv",std::ios_base::app);  //add data
    ofs << (ros::Time::now() - start_time_).toSec() << ",";
    for(int i=0;i<_tracer_data.position.size()-1;++i){
      ofs << _tracer_data.position[i] << ",";
      std::cout << _tracer_data.position[i] << ",";
    }
    ofs << _tracer_data.position[_tracer_data.position.size()-1] << std::endl;
    std::cout<< _tracer_data.position[_tracer_data.position.size()-1] << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"upper_recorder_node");
  ros::NodeHandle nh;

  std::string file_name;

  if(argc != 1) file_name=argv[1];
  else file_name="";

  TracerRecording tr(nh,file_name);

  ros::spin();

  return 0;
}
