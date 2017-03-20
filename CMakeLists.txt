cmake_minimum_required(VERSION 2.8.3)
project(obstacle_extraction)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  roscpp
  rospy
  pcl_conversions
  std_msgs
  tf
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system )
find_package(OpenCV 2.4.8 REQUIRED)
find_package(PCL 1.7 REQUIRED)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES obstacle_extraction
#  CATKIN_DEPENDS cv_bridge image_geometry image_transport roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(obstacle_extraction src/obstacle_extraction.cpp)
target_link_libraries(obstacle_extraction ${catkin_LIBRARIES} ${PCL_LIBRARIES})
