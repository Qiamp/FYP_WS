cmake_minimum_required(VERSION 2.8.3)
project(my_offboard_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros_msgs
  geometry_msgs
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp)
message(Eigen: ${EIGEN3_INCLUDE_DIR})

catkin_package()

include_directories(
${catkin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${YAML_CPP_INCLUDE_DIRS}
include
)

add_executable(offb_node src/offb_node.cpp)
target_link_libraries(offb_node ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
