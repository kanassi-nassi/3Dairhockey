cmake_minimum_required(VERSION 3.0.2)
project(bouncing_object_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  gazebo_msgs
  std_msgs
  gazebo_plugins
)

find_package(gazebo REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(cable_plugin SHARED src/cable_plugin.cpp)
target_link_libraries(cable_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_executable(bouncing_object_control src/bouncing_object_control.cpp)
target_link_libraries(bouncing_object_control ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})



