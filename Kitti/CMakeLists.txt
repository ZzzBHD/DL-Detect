cmake_minimum_required(VERSION 2.8.3)
project(kitti)

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
## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
  LIBRARIES kinect
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport jsk_recognition_msgs
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/kitti.cpp
# )

add_executable(box_show
  src/box_show.cpp
)
add_dependencies(box_show ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(box_show
  ${catkin_LIBRARIES}
)