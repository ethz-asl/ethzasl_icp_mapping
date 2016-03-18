cmake_minimum_required(VERSION 2.4.6)
project(ethzasl_icp_mapper)

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

#generate_messages()

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
# - N2351: shared pointers in standard
# - N2672: Initializer lists (gcc)
# - N1984: auto-typed variables (gcc, clang)


#############
# LIBRARIES #
#############


############
# BINARIES #
############
cs_add_executable(matcher_service
  src/matcher_service.cpp
)
target_link_libraries(matcher_service ${EXTRA_LIBS} rt yaml-cpp)

cs_add_executable(occupancy_grid_builder
  src/occupancy_grid_builder.cpp
)
target_link_libraries(occupancy_grid_builder ${EXTRA_LIBS} yaml-cpp)


#include_directories(${Boost_INCLUDE_DIRS}); 

cs_add_executable(mapper
  src/mapper.cpp
)
target_link_libraries(mapper ${catkin_LIBRARIES})

cs_add_executable(map_maintainer
  src/map_maintainer.cpp
)
target_link_libraries(map_maintainer ${catkin_LIBRARIES})

#cs_add_executable(interact_mapper
#  src/interact_mapper.cpp
#)
#target_link_libraries(interact_mapper ${catkin_LIBRARIES})

cs_add_executable(dynamic_mapper
  src/dynamic_mapper.cpp
)
target_link_libraries(dynamic_mapper ${catkin_LIBRARIES})

cs_add_executable(dynamic_trails
  src/dynamic_trails.cpp
)
target_link_libraries(dynamic_trails ${catkin_LIBRARIES})



##########
# EXPORT #
##########
cs_install()
cs_export()

