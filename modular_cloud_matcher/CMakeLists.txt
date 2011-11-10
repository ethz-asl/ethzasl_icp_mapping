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

rosbuild_genmsg()
rosbuild_gensrv()

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
	find_library(NABO_LIBRARY nabo
		/usr/local/lib
		/usr/lib
	)
	include_directories(${NABO_INCLUDE_DIR})

	find_path(POINT_MATCHER_INCLUDE_DIR pointmatcher/PointMatcher.h
		/usr/local/include
		/usr/include
	)
	find_library(POINT_MATCHER_LIBRARY pointmatcher
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

# look for yaml-cpp library
find_path(yaml-cpp_INCLUDE_DIRS yaml-cpp/yaml.h
	/usr/local/include
	/usr/include
)
find_library(yaml-cpp_LIBRARIES yaml-cpp PATHS
	/usr/local/lib
	/usr/lib
NO_DEFAULT_PATH)
if (yaml-cpp_INCLUDE_DIRS AND yaml-cpp_LIBRARIES)
	include_directories(${yaml-cpp_INCLUDE_DIRS})
	set(EXTRA_LIBS ${EXTRA_LIBS} ${yaml-cpp_LIBRARIES})
	add_definitions(-DHAVE_YAML_CPP)
endif (yaml-cpp_INCLUDE_DIRS AND yaml-cpp_LIBRARIES)

rosbuild_add_boost_directories()
rosbuild_add_executable(cloud_matcher_node src/modular_cloud_matcher.cpp)
rosbuild_add_executable(cloud_matcher_service src/modular_cloud_matcher_service.cpp)
rosbuild_add_executable(cloud_matcher_batch src/modular_cloud_matcher_batch.cpp)
rosbuild_add_executable(cloud_matcher_bag src/modular_cloud_matcher_bag.cpp)
if (FETCH_DEPENDENCIES)
	target_link_libraries(cloud_matcher_node pointmatcher nabo ${EXTRA_LIBS} rt)
	target_link_libraries(cloud_matcher_service pointmatcher nabo ${EXTRA_LIBS} rt)
	target_link_libraries(cloud_matcher_batch pointmatcher nabo ${EXTRA_LIBS} rt)
	target_link_libraries(cloud_matcher_bag pointmatcher nabo ${EXTRA_LIBS} argtable2 rt)
else (FETCH_DEPENDENCIES)
	target_link_libraries(cloud_matcher_node ${POINT_MATCHER_LIBRARY} ${NABO_LIBRARY} ${EXTRA_LIBS} rt)
	target_link_libraries(cloud_matcher_service ${POINT_MATCHER_LIBRARY} ${NABO_LIBRARY}  ${EXTRA_LIBS} rt)
	target_link_libraries(cloud_matcher_batch ${POINT_MATCHER_LIBRARY} ${NABO_LIBRARY} ${EXTRA_LIBS} rt)
	target_link_libraries(cloud_matcher_bag ${POINT_MATCHER_LIBRARY} ${NABO_LIBRARY} ${EXTRA_LIBS} argtable2 rt)
endif (FETCH_DEPENDENCIES)
