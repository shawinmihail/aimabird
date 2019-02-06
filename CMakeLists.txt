cmake_minimum_required(VERSION 2.8.3)
project(aimabird)

add_definitions(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# GAZEBO

find_package(gazebo REQUIRED)
set(PROTOBUF_IMPORT_DIRS "")

foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

if("${GAZEBO_VERSION}" VERSION_LESS "6.0")
  message(FATAL_ERROR "You need at least Gazebo 6.0. Your version: ${GAZEBO_VERSION}")
else()
  message("Gazebo version: ${GAZEBO_VERSION}")
endif()

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
)

###

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

add_executable(aimabird_control src/start_control.cpp)
add_executable(aimabird_replay src/replay.cpp)
add_executable(aimabird_planner src/start_planner.cpp)
add_executable(aimabird_test src/start_test.cpp)
add_library(aimabird src/AttControl.cpp)
add_library(estimation_test src/EstimationTest.cpp)

target_link_libraries(
    aimabird_control aimabird ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
)
target_link_libraries(
    aimabird_replay aimabird ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
)
target_link_libraries(
    aimabird_planner aimabird ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
)
target_link_libraries(
    aimabird_test estimation_test ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
)
