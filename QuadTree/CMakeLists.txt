#Making sure, that this script doesn't run on anything outdated
#Version 3.13 has the CMP0079 set to true
cmake_minimum_required(VERSION 3.13)

#Setting the variable responsible for the project name
set(PROJECT_NAME "Collisions")
project (${PROJECT_NAME})

#Adding the contained quadtree library
add_library(
	ContainedQuadTree 
	"${CMAKE_SOURCE_DIR}/QuadTree/ContainedQuadTree.h"
)

#Adding the quadtree library
add_library(
	QuadTree 
	"${CMAKE_SOURCE_DIR}/QuadTree/QuadTree.h"
)

#Giving the path to the needed includes
target_include_directories(ContainedQuadTree PUBLIC "${CMAKE_SOURCE_DIR}/QuadTree")
target_include_directories(QuadTree PUBLIC "${CMAKE_SOURCE_DIR}/QuadTree")
