cmake_minimum_required(VERSION 2.8.3)

include(ExternalProject)

ExternalProject_Add( foreignproject
  PREFIX foreignprojectlib
  GIT_REPOSITORY foreignproject.git
  BUILD_IN_SOURCE
  BUILD_COMMAND cmake --build .
  INSTALL_COMMAND ""
)

project(pointmatcher_ros)

find_package(catkin REQUIRED COMPONENTS nabo)


# c++ 0x
#if("${CMAKE_CXX_COMPILER_ID}" MATCHES "clang") 
#	set(CMAKE_COMPILER_IS_CLANGXX)
#endif ("${CMAKE_CXX_COMPILER_ID}" MATCHES "clang")
if(CMAKE_COMPILER_IS_GNUCXX)
	execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
	if (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
		add_definitions(-std=gnu++0x)
	else (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
		message(SEND_ERROR "You need GCC version 4.4 or greater to compile this library")
	endif (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
else(CMAKE_COMPILER_IS_GNUCXX)
	message(SEND_ERROR "You need partial C++0x support for N2351, N2672, N1984. Currently this build toolchain supports only GCC >= 4.4. If your compiler supports these specifications, please send us a patch.")
endif(CMAKE_COMPILER_IS_GNUCXX)
# well, this breaks libpointmatcher on non-unix platform,
# please contribute the define for your favorite compiler
# It must support (compiler support in parenthesis):
# - N1984: auto-typed variables (gcc, clang)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(roscpp)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

add_definitions(-DHAVE_YAML_CPP)

rosbuild_add_boost_directories()
rosbuild_add_library(pointmatcher_ros src/point_cloud.cpp src/transform.cpp)
