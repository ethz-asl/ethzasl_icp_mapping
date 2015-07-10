cmake_minimum_required(VERSION 2.8.3)
project(ethzasl_icp_mapper)

find_package(catkin REQUIRED roscpp rospy rosbag std_msgs geometry_msgs nav_msgs
tf tf2 tf_conversions message_filters map_msgs libpointmatcher_ros visualization_msgs
interactive_markers ethzasl_gridmap_2d message_generation libpointmatcher)

add_service_files( FILES
CorrectPose.srv
GetBoundedMap.srv
GetMode.srv
LoadMap.srv
MatchClouds.srv
SetMode.srv
)

generate_messages(DEPENDENCIES sensor_msgs nav_msgs std_msgs geometry_msgs actionlib_msgs)

catkin_package (
	CATKIN_DEPENDS roscpp rospy rosbag std_msgs geometry_msgs nav_msgs
tf tf2 tf_conversions message_filters map_msgs libpointmatcher_ros ethzasl_gridmap_2d visualization_msgs interactive_markers
message_runtime libpointmatcher
)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE RelWithDebInfo)

# c++ 0x
execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
add_definitions(-std=gnu++0x)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_definitions(-DHAVE_YAML_CPP)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(matcher_service src/matcher_service.cpp)
add_dependencies(matcher_service ${PROJECT_NAME}_gencpp)
target_link_libraries(matcher_service ${catkin_LIBRARIES})

add_executable(occupancy_grid_builder src/occupancy_grid_builder.cpp)
add_dependencies(occupancy_grid_builder ${PROJECT_NAME}_gencpp)
target_link_libraries(occupancy_grid_builder ${catkin_LIBRARIES} ${EXTRA_LIBS})

add_executable(mapper src/mapper.cpp)
add_dependencies(mapper ${PROJECT_NAME}_gencpp)
target_link_libraries(mapper ${catkin_LIBRARIES})

add_executable(map_maintainer src/map_maintainer.cpp )
add_dependencies(map_maintainer ${PROJECT_NAME}_gencpp)
target_link_libraries(map_maintainer ${catkin_LIBRARIES})

add_executable(interact_mapper src/interact_mapper.cpp )
add_dependencies(interact_mapper ${PROJECT_NAME}_gencpp)
target_link_libraries(interact_mapper ${catkin_LIBRARIES})

add_executable(dynamic_mapper src/dynamic_mapper.cpp )
add_dependencies(dynamic_mapper ${PROJECT_NAME}_gencpp)
target_link_libraries(dynamic_mapper ${catkin_LIBRARIES})

add_executable(dynamic_trails src/dynamic_trails.cpp )
add_dependencies(dynamic_trails ${PROJECT_NAME}_gencpp)
target_link_libraries(dynamic_trails ${catkin_LIBRARIES})
