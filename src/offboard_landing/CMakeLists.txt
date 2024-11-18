cmake_minimum_required(VERSION 3.0.2)
project(offboard_landing)

## 查找catkin的依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mavros_msgs
)

## 声明catkin包
catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs mavros_msgs
)

## 包含头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 添加可执行文件
add_executable(offboard_landing src/offboard_landing.cpp)

## 链接catkin包的库
target_link_libraries(offboard_landing
  ${catkin_LIBRARIES}
)

## 添加依赖关系
add_dependencies(offboard_landing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})