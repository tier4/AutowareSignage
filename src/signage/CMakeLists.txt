cmake_minimum_required(VERSION 2.8.3)
project(signage)

find_package(catkin REQUIRED COMPONENTS
  rospy
  autoware_perception_msgs
  autoware_vehicle_msgs
  pacmod_msgs
  autoware_system_msgs
  rosgraph_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/signage_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY
  resource
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
