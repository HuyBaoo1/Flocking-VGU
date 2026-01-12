
cmake_minimum_required(VERSION 3.8.0)
project(footbot_flocking)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

set(CMAKE_MACOSX_RPATH 0)

find_package(PkgConfig)
find_package(ARGoS REQUIRED)
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR})

add_library(footbot_flocking MODULE 
    footbot_flocking.h 
    footbot_flocking.cpp
)
# import argos libraries required for flocking 
target_link_libraries(footbot_flocking
    argos3core_simulator
    argos3plugin_simulator_footbot
    argos3plugin_simulator_genericrobot
)
