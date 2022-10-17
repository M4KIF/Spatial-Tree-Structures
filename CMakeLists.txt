#Checking for correct cmake version to suit my needs
cmake_minimum_required(VERSION 3.12)

#Setting the name, compilator flags and c++ version check
project(Spatial-Tree-Structures VERSION 1.0.0)

add_subdirectory(Dependencies/Collisions-library)
add_subdirectory(Octree)
add_subdirectory(QuadTree)
