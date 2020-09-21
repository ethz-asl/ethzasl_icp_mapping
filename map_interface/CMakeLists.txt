cmake_minimum_required(VERSION 2.8.3)
project(map_interface)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_package()

cs_add_library(${PROJECT_NAME}
        src/map_interface.cc)

add_executable(map_interface_node src/map_interface_node.cc)
target_link_libraries(map_interface_node ${catkin_LIBRARIES} ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
