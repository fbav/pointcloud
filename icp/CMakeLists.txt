cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(icp_align_demo)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (align_demo align_demo.cpp)
target_link_libraries (align_demo ${PCL_LIBRARIES})