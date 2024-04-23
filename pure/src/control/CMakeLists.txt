cmake_minimum_required(VERSION 3.0.2)
project(control)

find_package(catkin REQUIRED)

catkin_package(
  # CATKIN_DEPENDS other_catkin_pkg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Mark other files for installation (e.g., launch and config files, etc.)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

