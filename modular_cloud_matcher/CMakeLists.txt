cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# dependencies

if (FETCH_DEPENDENCIES)
	# eigen is provided as a ROS package in geometry
	# # eigen
	# find_path(EIGEN2_INCLUDE_DIR Eigen/Core
	# 	/usr/local/include/eigen2
	# 	/usr/include/eigen2
	# )
	# include_directories(${EIGEN2_INCLUDE_DIR})

	# Build nabo
	execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.nabo RESULT_VARIABLE _make_failed)
	if(_make_failed)
	message(FATAL_ERROR "Build of Nabo failed")
	endif(_make_failed)

	# Link nabo
	include_directories(${PROJECT_SOURCE_DIR}/nabo/include)
	link_directories(${PROJECT_SOURCE_DIR}/nabo/lib)


	# Build pointmatcher
	execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.pointmatcher RESULT_VARIABLE _make_failed)
	if(_make_failed)
	message(FATAL_ERROR "Build of Pointmatcher failed")
	endif(_make_failed)

	# Link pointmatcher
	include_directories(${PROJECT_SOURCE_DIR}/pointmatcher/include)
	link_directories(${PROJECT_SOURCE_DIR}/pointmatcher/lib)
else (FETCH_DEPENDENCIES)
	find_path(NABO_INCLUDE_DIR nabo/nabo.h
		/usr/local/include
		/usr/include
	)
	find_library(NABO_LIBRARY libnabo
		/usr/local/lib
		/usr/lib
	)
	include_directories(${NABO_INCLUDE_DIR})

	find_path(POINT_MATCHER_INCLUDE_DIR pointmatcher/PointMatcher.h
		/usr/local/include
		/usr/include
	)
	find_library(POINT_MATCHER_LIBRARY libpointmatcher
		/usr/local/lib
		/usr/lib
	)
	include_directories(${POINT_MATCHER_INCLUDE_DIR})
endif (FETCH_DEPENDENCIES)

# look for OpenCL library
if (WIN32)
	find_library(OPENCL_LIBRARIES opencl64)
	if (!OPENCL_LIBRARIES)
		find_library(OPENCL_LIBRARIES opencl32)
	endif (!OPENCL_LIBRARIES)
else (WIN32)
	find_library(OPENCL_LIBRARIES OpenCL ENV LD_LIBRARY_PATH)
endif (WIN32)
# if found, add
if (OPENCL_LIBRARIES)
	set(EXTRA_LIBS ${OPENCL_LIBRARIES})
endif (OPENCL_LIBRARIES)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_boost_directories()
rosbuild_add_executable(cloud_matcher_node src/modular_cloud_matcher.cpp)
if (FETCH_DEPENDENCIES)
	target_link_libraries(cloud_matcher_node pointmatcher nabo ${EXTRA_LIBS})
else (FETCH_DEPENDENCIES)
	target_link_libraries(cloud_matcher_node ${POINT_MATCHER_LIBRARY} ${NABO_LIBRARY} ${EXTRA_LIBS})
endif (FETCH_DEPENDENCIES)
