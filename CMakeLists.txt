cmake_minimum_required(VERSION 3.5.1)
project(aliengo_planner)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(
        CATKIN_PACKAGE_DEPENDENCIES
        roscpp
        grid_map_msgs
)

find_package(catkin REQUIRED
        COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        )

###########
# MSGs #
###########

catkin_package(
        INCLUDE_DIRS
        config
        include
        CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
)

include_directories(
        SYSTEM
        config
        include
        ${catkin_INCLUDE_DIRS}
)

##############
# Executable #
##############

add_executable(planner src/planner.cpp
        )

target_link_libraries(planner ${catkin_LIBRARIES}
        )

###########
## Clang ##
###########

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
    add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
