cmake_minimum_required(VERSION 2.8.3)
project(motion_tracer)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  tf
  seed_smartactuator_sdk
  seed_r7_ros_controller
  seed_r7_robot_interface
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# add c++ dependencies :
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c+11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  add_definitions(-DCXX11_SUPPORTED)
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCXX11_SUPPORTED)
else()
  message(FATAL "c++11 required but not supported")
endif()

# add boost dependencies :
find_package(Boost REQUIRED COMPONENTS filesystem system signals thread)
if(NOT Boost_INCLUDE_DIRS)
  message(FATAL "boost required but not supported")
else()
  include_directories(include ${Boost_INCLUDE_DIRS})
endif()

#for service and client----
# add_service_files(
#  FILES
# )
#  generate_messages(
#    DEPENDENCIES
#  )
#--------------------------

#for dynamic reconfigure--
generate_dynamic_reconfigure_options(
  cfg/Robot.cfg
  cfg/Tracer.cfg
)
#-------------------------
catkin_package()

###########
## Build ##
###########

include_directories(  ${catkin_INCLUDE_DIRS})

add_executable(tracer_teleop_node
  src/tracer_teleop.cpp
  src/tracer_command.cpp
)
target_link_libraries(tracer_teleop_node ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS})
add_dependencies(tracer_teleop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#controller
add_executable(upper_controller_node src/upper_controller.cpp)
target_link_libraries(upper_controller_node ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS})
add_dependencies(upper_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lower_controller_node src/lower_controller.cpp)
target_link_libraries(lower_controller_node ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS})
add_dependencies(lower_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#recording
add_executable(tracer_recording_node src/tracer_recording.cpp)
target_link_libraries(tracer_recording_node ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS})
add_dependencies(tracer_recording_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(joy_recording_node src/joy_recording.cpp)
target_link_libraries(joy_recording_node ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS})
add_dependencies(joy_recording_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#playback
add_executable(tracer_playback_node src/tracer_playback.cpp)
target_link_libraries(tracer_playback_node ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS})
add_dependencies(tracer_playback_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(joy_playback_node src/joy_playback.cpp)
target_link_libraries(joy_playback_node ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS})
add_dependencies(joy_playback_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
