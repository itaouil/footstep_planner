cmake_minimum_required(VERSION 3.5.1)
project(aliengo_navigation)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(
        CATKIN_PACKAGE_DEPENDENCIES
        tf
        roscpp
        grid_map_ros
        grid_map_msgs
        grid_map_core
        tf_conversions
        message_filters
)

find_package(catkin REQUIRED
        COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        )

find_package(Boost REQUIRED COMPONENTS system)

###########
# MSGs #
###########

catkin_package(
        INCLUDE_DIRS
        config
        include
        CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        DEPENDS
        Boost
)

include_directories(
        config
        include
        SYSTEM
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

##############
# Executable #
##############

add_executable(navigation src/navigation.cpp src/planner.cpp src/search/AStar.cpp
        )

target_link_libraries(navigation ${catkin_LIBRARIES}
        )

###########
## Clang ##
###########

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
    add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
