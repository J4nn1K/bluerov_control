cmake_minimum_required(VERSION 3.0.2)
project(bluerov_control)

find_package(catkin REQUIRED COMPONENTS
  hippocampus_common
  hippocampus_msgs
  dynamic_reconfigure
  mavros_msgs
)

catkin_python_setup()

## Generate dynamic reconfigure parameters in the 'config' folder
generate_dynamic_reconfigure_options(
  config/PidControl.cfg
  config/DepthControl.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)