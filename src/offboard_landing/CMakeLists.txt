cmake_minimum_required(VERSION 3.0.2)
project(offboard_landing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mavros_msgs
  eigen_conversions
  tf2_eigen
)

find_package(Eigen3 REQUIRED)
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES offboard_landing
  CATKIN_DEPENDS roscpp geometry_msgs mavros_msgs
)

add_executable(offboard_landing_node 
  src/offboard_landing.cpp
)
target_link_libraries(offboard_landing_node
  ${catkin_LIBRARIES}
)

install(TARGETS offboard_landing_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)