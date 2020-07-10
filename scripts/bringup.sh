#!/bin/bash
ros_ws=$1
robot_ip=$2
user_name=$3
password=$4
local_ip=$5
command=$6

#ros_ws=/home/seed/ros/kinetic
#ros_ws=/home/seed/ros/melodic
source ${ros_ws}/devel/setup.bash
roscd motion_tracer/scripts

if [ ${command} = "controller" ]; then
  dual_shock=$7

  expect ssh.exp ${robot_ip} ${password} "killall gnome-terminal-server;exit";
  gnome-terminal --zoom=0.5 \
    --tab -e 'bash -c "expect ssh.exp '${robot_ip}' '${password}' \"export ROS_IP='${robot_ip}'\" \"roslaunch motion_tracer robot_bringup.launch DUALSHOCK:='${dual_shock}' \" "';

elif [ ${command} = "tracer" ]; then
  device=$7
  dual_shock=$8
  initial_pose=$9
  neck_movement=${10}
  neck_offset=${11}
  neck_auto=${12}
  neck_reverse=${13}

  export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
  if [ ${device} = "mechaless" ]; then
    :
  elif [ ${device} = "mechanical" ]; then
    gnome-terminal --zoom=0.5 --tab -e 'bash -c "roslaunch --wait motion_tracer tracer_bringup.launch \\
      DUALSHOCK:='${dual_shock}' initial_pose:='${initial_pose}' neck_movement:='${neck_movement}' neck_offset:='${neck_offset}' neck_auto:='${neck_auto}' neck_reverse:='${neck_reverse}' "';
  fi

elif [ ${command} = "rviz" ]; then
  export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
  gnome-terminal --zoom=0.5 --tab -e 'bash -c "roslaunch motion_tracer view.launch"'

elif [ ${command} = "path" ]; then
  export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
  echo 'ROS_MASTER_URI is ' $ROS_MASTER_URI
  echo 'ROS_IP is ' $ROS_IP

elif [ ${command} = "record" ]; then
  argument=$7
  file_sufix=$8
  if [ ${argument} = "start" ]; then
    export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
    gnome-terminal --zoom=0.5 --tab -e 'bash -c "roslaunch motion_tracer recording.launch file_sufix:='${file_sufix}';exit "'

  elif [ ${argument} = "stop" ]; then
    export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
    rosnode kill tracer_recording_node&
    rosnode kill joy_recording_node
  fi

elif [ ${command} = "play" ]; then
  file_sufix=$7
  export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
  gnome-terminal --zoom=0.5 --tab -e 'bash -c "roslaunch motion_tracer playback.launch file_sufix:='${file_sufix}';exit "'

elif [ ${command} = "initial_pose" ]; then
  initial_pose=$7
  export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
  rosrun dynamic_reconfigure dynparam set /tracer_teleop_node 'initial_pose' ${initial_pose}
  rosparam set /initial_pose ${initial_pose}

elif [ ${command} = "neck_pose" ]; then
  neck_movement=$7
  neck_auto=$8
  neck_reverse=$9
  neck_offset=${10}

  export ROS_MASTER_URI=http://${robot_ip}:11311;export ROS_IP=${local_ip};
  rosrun dynamic_reconfigure dynparam set upper_controller_node \
    "{'movement':'${neck_movement}', 'automatically':${neck_auto}, 'reverse':${neck_reverse}, 'offset':${neck_offset}}" 

elif [ ${command} = "kill" ]; then
  killall gnome-terminal-server&
  expect ssh.exp ${user_name}@${robot_ip} ${password} "export DISPLAY=:0.0;killall gnome-terminal-server; bash '${ros_ws}'/src/motion_tracer/scripts/teleop_mover.sh"
fi
