cmake_minimum_required(VERSION 3.0.2)
project(bluerov_control)

find_package(catkin REQUIRED COMPONENTS
  hippocampus_common
  hippocampus_msgs
  dynamic_reconfigure
  mavros_msgs
  message_generation
)

catkin_python_setup()

## Generate dynamic reconfigure parameters in the 'config' folder
generate_dynamic_reconfigure_options(
  cfg/DepthControl.cfg
  cfg/ThrusterTest.cfg
)

add_message_files(
  FILES
  Configuration.msg
  ControllerErrors.msg
  PathTargets.msg
)

add_service_files(
  FILES
  Bezier.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)