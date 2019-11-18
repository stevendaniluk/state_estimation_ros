cmake_minimum_required(VERSION 2.8.3)
project(state_estimation_ros)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include state_estimation/include
  LIBRARIES state_estimation state_estimation_ros
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include
  state_estimation/include
)

###########
## Build ##
###########

# Add our core state estimation libraries
add_subdirectory(state_estimation)

# ROS wrappers library
add_library(state_estimation_ros
  src/ros_conversions.cpp
)
target_link_libraries(state_estimation_ros
  state_estimation
  ${catkin_LIBRARIES}
)

# Install the library and header files
install(TARGETS state_estimation_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

# Unit test for ros conversions
add_executable(test_ros_conversions test/test_ros_conversions.cpp)
add_test(NAME test_ros_conversions COMMAND test_ros_conversions)
target_link_libraries(test_ros_conversions
  gtest_main
  state_estimation_ros
)
