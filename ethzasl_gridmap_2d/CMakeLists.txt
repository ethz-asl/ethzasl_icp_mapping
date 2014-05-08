cmake_minimum_required(VERSION 2.8.3)
project(ethzasl_gridmap_2d)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs)

catkin_package(
	CATKIN_DEPENDS roscpp nav_msgs
	DEPENDS eigen
)

include_directories(include)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

add_library(ethzasl_gridmap_2d src/grid-map.cpp)
