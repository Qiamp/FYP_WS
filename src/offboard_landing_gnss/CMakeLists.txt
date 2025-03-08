cmake_minimum_required(VERSION 3.0.2)
project(offboard_landing_gnss)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mavros_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(offboard_landing_gnss 
  src/offboard_landing_gnss.cpp
)

target_link_libraries(offboard_landing_gnss
  ${catkin_LIBRARIES}
)

install(TARGETS offboard_landing_gnss
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)