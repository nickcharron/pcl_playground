cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(iterative_closest_point)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (iterative_closest_point src/iterative_closest_point.cpp)
add_executable (incremental_iterative_closest_point src/incremental_iterative_closest_point.cpp)
add_executable (calibrate_lidars_from_model src/calibrate_lidars_from_model.cpp)
add_executable (cloud_viewer src/cloud_viewer.cpp)
target_link_libraries (cloud_viewer ${PCL_LIBRARIES})
target_link_libraries (iterative_closest_point ${PCL_LIBRARIES})
target_link_libraries (incremental_iterative_closest_point ${PCL_LIBRARIES})
target_link_libraries (calibrate_lidars_from_model ${PCL_LIBRARIES})
