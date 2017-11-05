################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(candynamix_rviz)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  turtlebot3_msgs
)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES candynamix_rviz_node
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf candynamix_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(candynamix_rviz_node src/candynamix_rviz.cpp)
target_link_libraries(candynamix_rviz_node ${catkin_LIBRARIES})
add_dependencies(candynamix_rviz_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

################################################################################
# Install
################################################################################
install(TARGETS candynamix_rviz_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
