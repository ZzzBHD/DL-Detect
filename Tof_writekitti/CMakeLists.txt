cmake_minimum_required(VERSION 2.8.3)
project(tof_writekitti)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
   message_generation std_msgs roscpp sensor_msgs cv_bridge image_transport pcl_ros pcl_conversions
)
find_package( OpenCV REQUIRED )
find_package( PCL 1.9 REQUIRED )
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

catkin_package(
  LIBRARIES kinect
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport
)


add_library(lib
  src/lib.cpp
)
add_dependencies(lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lib
  ${catkin_LIBRARIES}
)


add_executable(tof_writekitti
  src/tof_writekitti.cpp
  src/svm.cpp
  src/plane_fitting_ground_segmenter.cpp
  src/lib.cpp
  src/object-svm.cpp
)
add_dependencies(tof_writekitti ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tof_writekitti
  ${catkin_LIBRARIES}
)
