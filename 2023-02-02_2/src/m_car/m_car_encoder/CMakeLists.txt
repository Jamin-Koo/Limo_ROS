cmake_minimum_required(VERSION 3.0.2)
project(m_car_encoder)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2_geometry_msgs
  sensor_msgs
)
find_package(OpenCV REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msg geometry_msgs tf2_geometry_msgs sensor_msgs tf
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(

  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_m_car_control_node src/car_control_node_encoder.cpp)


add_dependencies(ros_m_car_control_node ${ros_m_car_control_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ros_m_car_control_node ${catkin_LIBRARIES} )


