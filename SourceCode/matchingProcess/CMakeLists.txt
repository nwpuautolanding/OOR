cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
 
#find_package(PCL 1.7 REQUIRED)
#find_package(OpenCV REQUIRED)
 
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
 
 
add_executable(graphTest src/mainTest.cpp src/graph.c)
 
#target_link_libraries (planeExtraction ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
