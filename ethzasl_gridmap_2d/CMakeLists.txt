cmake_minimum_required(VERSION 2.8.3)
project(ethzasl_gridmap_2d)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs eigen)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp nav_msgs
	DEPENDS eigen
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include ${eigen_INCLUDE_DIRS})

add_library(ethzasl_gridmap_2d src/grid-map.cpp)
