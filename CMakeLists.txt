cmake_minimum_required(VERSION 2.8.3)
project(aimabird)

add_definitions(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# GAZEBO

set(PROTOBUF_IMPORT_DIRS "")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")


catkin_package()

include_directories ( /usr/include/eigen3 )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(aimabird_control src/start_control.cpp)
# add_executable(aimabird_replay src/replay.cpp)
# add_executable(aimabird_planner src/start_planner.cpp)
# add_executable(aimabird_test src/start_test.cpp)
add_library(aimabird src/AttControl.cpp)
# add_library(estimation_test src/EstimationTest.cpp)

target_link_libraries(
    aimabird_control aimabird ${catkin_LIBRARIES}
)
# target_link_libraries(
#     aimabird_replay aimabird ${catkin_LIBRARIES}
# )
# target_link_libraries(
#     aimabird_planner aimabird ${catkin_LIBRARIES}
# )
# target_link_libraries(
#     aimabird_test estimation_test ${catkin_LIBRARIES}
# )
