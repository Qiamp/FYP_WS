cmake_minimum_required(VERSION 3.0.2)
project(vicon_mission)

# Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mavros_msgs
)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Catkin package configuration
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vicon_mission
 CATKIN_DEPENDS roscpp geometry_msgs mavros_msgs
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Add executable
add_executable(vicon_mission 
  src/vicon_mission.cpp
)

# Link libraries
target_link_libraries(vicon_mission
  ${catkin_LIBRARIES}
)

# Install targets
install(TARGETS vicon_mission
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)