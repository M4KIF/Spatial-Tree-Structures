#Checking for correct cmake version to suit my needs
cmake_minimum_required(VERSION 3.12)

#Setting the name, compilator flags and c++ version check
project(Spatial-Tree-Structures VERSION 1.0.0)

#Checking for git submodules updates, via checking the .gitmodules file and
#pulling the modules that are written in it.
find_package(Git QUIET)
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

#Searching for Collisions-library
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/Dependencies/Collisions-library/CMakeLists.txt")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()

#Adding subdirectory works when the subdir has a CMakeLists.txt in it
include_directories(Dependencies)

add_subdirectory(Dependencies/Collisions-library)

