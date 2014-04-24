cmake_minimum_required(VERSION 2.8.3)
project(libpointmatcher)
include(ExternalProject)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  #yaml-cpp
  nabo
  #eigen3
  #doxygen
)

SET(pointmatcher_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/upstream_src")
SET(pointmatcher_CMAKE_ARGS -DCMAKE_BUILD_TYPE=RelWithDebInfo 
	-DCMAKE_CXX_FLAGS="-fPIC" 
	-DNABO_INCLUDE_DIR=`rospack cflags-only-I libnabo` 
	-DCMAKE_LIBRARY_PATH=`rospack libs-only-L libnabo` 
    -DPYTHON_CUSTOM_TARGET=src)
#SET(pointmatcher_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

ExternalProject_Add(pointmatcher PREFIX ${pointmatcher_PREFIX} 
								 BUILD_IN_SOURCE
								 CMAKE_ARGS ${pointmatcher_CMAKE_ARGS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
  CATKIN_DEPENDS libnabo
  DEPENDS eigen yaml-cpp boost
)

