cmake_minimum_required(VERSION 2.8.3)
project(motor_ctrl)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/motor_ctrl_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
