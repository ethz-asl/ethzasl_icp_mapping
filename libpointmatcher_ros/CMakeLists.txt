cmake_minimum_required(VERSION 2.4.6)
project(libpointmatcher_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-DHAVE_YAML_CPP)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

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


#############
# LIBRARIES #
#############
cs_add_library(pointmatcher_ros
  src/point_cloud.cpp
  src/transform.cpp
)
target_link_libraries(pointmatcher_ros ${catkin_LIBRARIES})

############
# BINARIES #
############


##########
# EXPORT #
##########
cs_install()
cs_export()
