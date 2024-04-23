cmake_minimum_required(VERSION 3.0.2)
project(actuator_ctrl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  can_raw
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs can_raw 
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/actuator_ctrl_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

