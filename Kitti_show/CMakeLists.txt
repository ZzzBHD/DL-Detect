cmake_minimum_required(VERSION 2.8.3)
project(bag)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_definitions(-std=c++11)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
   message_generation std_msgs roscpp sensor_msgs cv_bridge image_transport pcl_ros pcl_conversions
   rospy
)
find_package( OpenCV REQUIRED )
find_package( PCL 1.3 REQUIRED )
find_package(Boost REQUIRED COMPONENTS
thread)
include_directories(
  include ${catkin_INCLUDE_DIRS}${OpenCV_INCLUDE_DIRS})
include_directories(
  include ${PCL_INCLUDE_DIRS})
include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include)
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate added messages and services with any dependencies listed here

#  generate_messages(
#    DEPENDENCIES
#    std_msgs  # Or other packages containing msgs
#  )

catkin_package(
  LIBRARIES kinect
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport jsk_recognition_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(
#   include ${catkin_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
#   include
# )

add_executable(kitti_show_c
  src/kitti_show_c.cpp
)
add_dependencies(kitti_show_c ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kitti_show_c
  ${catkin_LIBRARIES}
)

add_executable(test
  src/test.cpp
)
add_dependencies(test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test
  ${catkin_LIBRARIES}
)
