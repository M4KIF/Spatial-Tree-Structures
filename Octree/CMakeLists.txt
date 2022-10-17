#Making sure, that this script doesn't run on anything outdated
#Version 3.13 has the CMP0079 set to true
cmake_minimum_required(VERSION 3.13)

#Setting the variable responsible for the project name
set(PROJECT_NAME "Collisions")
project (${PROJECT_NAME})

#Adding the contained Octree library
add_library(
	ContainedOctree 
	"${CMAKE_SOURCE_DIR}/Octree/ContainedOctree.h"
)

#Adding the Octree library
add_library(
	Octree 
	"${CMAKE_SOURCE_DIR}/Octree/Octree.h"
)

#Giving the path to the needed includes
target_include_directories(ContainedOctree PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/Octree")
target_include_directories(Octree PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/Octree")
