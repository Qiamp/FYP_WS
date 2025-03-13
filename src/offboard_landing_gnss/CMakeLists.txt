cmake_minimum_required(VERSION 3.0.2)
project(offboard_landing_gnss)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mavros_msgs
  sensor_msgs
  nav_msgs
  geographic_msgs
  geodesy  # Added geodesy component
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES offboard_landing_gnss
  CATKIN_DEPENDS 
    roscpp 
    geometry_msgs 
    mavros_msgs 
    sensor_msgs
    nav_msgs 
    geographic_msgs
    geodesy  # Added to catkin depends
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(offboard_landing_gnss_node 
  src/offboard_landing_gnss.cpp
)

target_link_libraries(offboard_landing_gnss_node
  ${catkin_LIBRARIES}
)

install(TARGETS offboard_landing_gnss_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)