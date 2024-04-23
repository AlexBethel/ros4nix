cmake_minimum_required(VERSION 3.0.2)
project(hud)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  theora_image_transport
  can_convert
)

catkin_package(CATKIN_DEPENDS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  theora_image_transport
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(hud_node
  src/hud_node.cpp
)
add_dependencies(
  hud_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(hud_node ${catkin_LIBRARIES})

install(
  TARGETS hud_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
