cmake_minimum_required(VERSION 2.4.6)
project(ethzasl_gridmap_2d)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

#############
# LIBRARIES #
#############
cs_add_library(ethzasl_gridmap_2d
  src/grid-map.cpp
)
target_link_libraries(ethzasl_gridmap_2d
	${PROJECT_NAME}
)

############
# BINARIES #
############


##########
# EXPORT #
##########
cs_install()
cs_export()
