cmake_minimum_required(VERSION 2.8.3)
project(aimabird)

add_definitions(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES broadcast
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories ( /usr/include/eigen3 )

include_directories(    
  ${catkin_INCLUDE_DIRS}
)

add_executable(aimabird_node src/main.cpp)
add_library(aimabird src/AttControl.cpp)
#add_library(px4math src/Matrix/matrix/math.hpp)

target_link_libraries(
    aimabird_node aimabird ${catkin_LIBRARIES}
)
