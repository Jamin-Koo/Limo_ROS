cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} [OpenCV.cmake PATH])

project(opencv_lane_detect_topic)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  cv_bridge
  image_transport
  geometry_msgs
)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(OpenCV_INCLUDE_DIRS "/usr/include/opencv;/usr/include")

find_package(OpenCV REQUIRED)
 

catkin_package(
 # INCLUDE_DIRS include
  LIBRARIES opencv_lane_detect
  CATKIN_DEPENDS rospy roscpp std_msgs geometry_msgs
 # DEPENDS system_lib
  DEPENDS OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${OpenCV_LIB_DIR})
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/opencv_line_detection.cpp
# )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(vision_lane_node src/opencv_lane_detect_topic.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(vision_lane_node ${vision_lane_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(vision_lane_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)


