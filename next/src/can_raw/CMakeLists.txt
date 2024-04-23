cmake_minimum_required(VERSION 3.0.2)
project(can_raw)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

add_message_files(
  FILES
  CanFrame.msg
)
generate_messages(
  DEPENDENCIES
)

catkin_package(CATKIN_DEPENDS
  roscpp
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(can_raw_node
  src/can_raw_node.cpp
)
add_dependencies(
  can_raw_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(can_raw_node ${catkin_LIBRARIES})

install(
  TARGETS can_raw_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
