#Making sure, that this script doesn't run on anything outdated
cmake_minimum_required(VERSION 3.12)

#Setting the variable responsible for the project name
set(PROJECT_NAME "Collisions")
project (${PROJECT_NAME})

#Adding the contained Octree library
add_library(
	ContainedOctree 
	"${CMAKE_CURRENT_SOURCE_DIR}/Octree/ContainedOctree.h"
)

#Adding the Octree library
add_library(
	Octree 
	"${CMAKE_CURRENT_SOURCE_DIR}/Octree.h"
)

#Giving the path to the needed includes
target_include_directories(ContainedOctree PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/Octree")
target_include_directories(Octree PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/Octree")

#Searching whether git has been initialised here
find_package(Git QUIET)

#Proceeding to the submodules checking
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
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
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/Dependencies/Collisions-library/CMakeLists.txt")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()

#Adding the path to glm directory with a CMakeLists.txt inside
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/Collisions-library")

#Linking main library with the dependency
target_link_libraries(ContainedOctree PUBLIC aabb-library)
target_link_libraries(Octree PUBLIC aabb-library)

target_link_libraries(ContainedOctree PUBLIC Octree)
