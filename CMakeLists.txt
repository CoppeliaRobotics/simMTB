cmake_minimum_required(VERSION 3.15)
project(simMTB)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

find_package(Boost REQUIRED)

if(NOT COPPELIASIM_INCLUDE_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(COPPELIASIM_INCLUDE_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/include)
    else()
        set(COPPELIASIM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH ${COPPELIASIM_INCLUDE_DIR}/cmake)
find_package(CoppeliaSim 4.5.1.7 REQUIRED COMPONENTS simLib-1 simLib-socket-1 simLib-scriptFunctionData-1)

coppeliasim_add_plugin(
    simMTB
    LEGACY
    SOURCES
    sourceCode/simMTB.cpp
)
coppeliasim_add_lua(lua/simMTB.lua)
coppeliasim_add_lua(lua/simMTB-ce.lua)
