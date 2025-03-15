cmake_minimum_required(VERSION 3.0.2)
project(gps_converter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros_msgs
  nmea_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gps_to_rtcm_converter 
  src/gps_to_rtcm_converter.cpp
)

target_link_libraries(gps_to_rtcm_converter
  ${catkin_LIBRARIES}
)

install(TARGETS gps_to_rtcm_converter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)