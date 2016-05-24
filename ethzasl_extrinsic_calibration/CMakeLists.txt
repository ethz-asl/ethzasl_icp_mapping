cmake_minimum_required(VERSION 2.8.3)
project(ethzasl_extrinsic_calibration)

find_package(catkin REQUIRED COMPONENTS roscpp rospy tf)

find_package(Eigen3 QUIET)

# Eigen3 will not be found on Ubuntu 12.04, so we moke it
if(NOT EIGEN3_FOUND)
	# Fallback to cmake_modules
	find_package(Eigen REQUIRED)
	set(EIGEN3_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS})
	set(EIGEN_NAME "Eigen")
else()
	SET(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
	set(EIGEN_NAME "EIGEN3")
endif()
MESSAGE(STATUS "Found Eigen3 in ${EIGEN3_INCLUDE_DIRS}.")

catkin_package(
	CATKIN_DEPENDS roscpp rospy tf 
	DEPENDS ${EIGEN_NAME}
)


include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

add_executable(tf_logger src/tf_logger.cpp)
target_link_libraries(tf_logger ${catkin_LIBRARIES})
add_executable(optimize src/optimize.cpp)
target_link_libraries(tf_logger ${catkin_LIBRARIES})

# Mark executables and/or libraries for installation
install(TARGETS tf_logger optimize
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
