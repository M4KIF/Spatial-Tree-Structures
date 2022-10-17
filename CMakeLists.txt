#Checking for correct cmake version to suit my needs
#Version 3.13 has the CMP0079 set to true
cmake_minimum_required(VERSION 3.13)

#Setting the name, compilator flags and c++ version check
project(Spatial-Tree-Structures VERSION 1.0.0)

add_subdirectory(Octree)
add_subdirectory(QuadTree)

#Adding the path to glm directory with a CMakeLists.txt inside
add_subdirectory("${CMAKE_SOURCE_DIR}/Dependencies/Collisions-library")

#Searching whether git has been initialised here
find_package(Git QUIET)

#Proceeding to the submodules checking
if(GIT_FOUND AND EXISTS "${CMAKE_SOURCE_DIR}/.git")
    # Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

#Checking for Collisions-library submodule
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/Dependencies/Collisions-library/CMakeLists.txt")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()

#Giving the path to the needed includes
target_include_directories(aabb-library PUBLIC "${CMAKE_SOURCE_DIR}/Dependencies/Collisions-library")

#Linking main library with the dependency
target_link_libraries(ContainedOctree PUBLIC aabb-library)
target_link_libraries(Octree PUBLIC aabb-library)

target_link_libraries(ContainedOctree PUBLIC Octree)

#Giving the path to the needed includes
target_include_directories(aabb-library PUBLIC "${CMAKE_SOURCE_DIR}/Dependencies/Collisions-library/AABB")

#Linking main library with the dependency
target_link_libraries(ContainedQuadTree PUBLIC aabb-library)
target_link_libraries(QuadTree PUBLIC aabb-library)

target_link_libraries(ContainedQuadTree PUBLIC QuadTree)
