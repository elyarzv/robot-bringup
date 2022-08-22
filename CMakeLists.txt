cmake_minimum_required(VERSION 3.0.2)
project(phoenix1_bringup)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  phoenix_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uv_bringup
#  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

catkin_install_python(PROGRAMS
  scripts/run_bringup.bash
  scripts/wait_for_topics.bash
  scripts/wait_for_sensors.bash
  scripts/robot_environment_vars.bash
  scripts/slam_launch_manager.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
