cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(oqton_icp)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(oqton_icp oqton_icp.cpp)
target_link_libraries(oqton_icp ${PCL_LIBRARIES})