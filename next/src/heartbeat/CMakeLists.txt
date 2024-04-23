cmake_minimum_required(VERSION 3.0.2)
project(heartbeat)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(heartbeat_server_node src/heartbeat_server_node.cpp)
add_executable(heartbeat_client_node src/heartbeat_client_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(heartbeat_server_node ${catkin_LIBRARIES})
target_link_libraries(heartbeat_client_node ${catkin_LIBRARIES})

