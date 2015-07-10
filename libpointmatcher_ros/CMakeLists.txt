cmake_minimum_required(VERSION 2.8.3)

project(libpointmatcher_ros)

find_package(catkin REQUIRED COMPONENTS
roscpp
sensor_msgs
nav_msgs
tf
tf_conversions
eigen_conversions
libpointmatcher
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES pointmatcher_ros
CATKIN_DEPENDS
roscpp
sensor_msgs
nav_msgs
tf
tf_conversions
eigen_conversions
libpointmatcher
)

# c++ 0x
execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
add_definitions(-std=gnu++0x)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_definitions(-DHAVE_YAML_CPP)
include_directories(include)

add_library(pointmatcher_ros src/point_cloud.cpp src/transform.cpp)
include_directories(${catkin_INCLUDE_DIRS})
target_link_libraries(pointmatcher_ros ${catkin_LIBRARIES})
