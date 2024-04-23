cmake_minimum_required(VERSION 3.0.2)
project(leds)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  can_raw
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
add_executable(leds_node src/leds_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(leds_node ${catkin_LIBRARIES})

