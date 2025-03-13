cmake_minimum_required(VERSION 3.0.2)
project(apriltag_pose_pub)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  apriltag_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES apriltag_pose_pub
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs apriltag_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(apriltag_pose_pub_gnss
  src/apriltag_pose_pub.cpp
)

target_link_libraries(apriltag_pose_pub_gnss
  ${catkin_LIBRARIES}
)

install(TARGETS apriltag_pose_pub_gnss
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)