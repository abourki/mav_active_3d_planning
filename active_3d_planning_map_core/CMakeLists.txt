cmake_minimum_required(VERSION 2.8.3)
project(active_3d_planning_map_core)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14)

catkin_package()

cs_add_library(${PROJECT_NAME}
        src/map/mapping.cpp
        src/initialization/mapping_package.cpp
        )

cs_install()
cs_export()
