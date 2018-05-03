cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(supervoxel_clustering)
set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package(PCL 1.8 REQUIRED)
# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# eigen 
include_directories( "/usr/include/eigen3/" )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (supervoxel_clustering supervoxel_clustering.cpp)
target_link_libraries (supervoxel_clustering ${OpenCV_LIBS} ${PCL_LIBRARIES}) 
