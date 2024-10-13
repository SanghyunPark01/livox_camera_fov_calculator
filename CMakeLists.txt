cmake_minimum_required(VERSION 3.0.2)
project(livox_camera_fov_calculator)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  livox_ros_driver
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES OpenCV
  CATKIN_DEPENDS roscpp cv_bridge sensor_msgs 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_calc src/calc.cpp)
target_link_libraries(${PROJECT_NAME}_calc
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(run
  src/main.cpp
)
target_link_libraries(run
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}_calc
)

