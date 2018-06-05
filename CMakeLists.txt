cmake_minimum_required(VERSION 2.6)
project(EIBA)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

message(STATUS "BUILD TYPE:" ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -w")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -msse2 -msse3 -msse4")
include_directories(PlatformIndependence)

add_definitions(-DCFG_DEPTH_MAP -DCFG_GROUND_TRUTH -DCFG_TUNE_PARAMETERS)

option(YAML_IO "YAML_IO" ON)

# enable YAML IO
if (YAML_IO)
    message("Enable Yaml IO")
    add_definitions(-DENABLE_YAML_IO)
    find_package(yaml-cpp REQUIRED)
    include_directories(${YAML_CPP_INCLUDE_DIR})
endif()


include_directories(
        .
        BundleAdjustment
        Geometry
        ImageTracking
        LinearAlgebra
        Map
        Math
        Utility)

add_subdirectory(BundleAdjustment)
add_subdirectory(Geometry)
add_subdirectory(Map)
add_subdirectory(Utility)

set(EIBA_LINK_LIBS
        ${EIBA_LINK_LIBS}
        BundleAdjustment
        Geometry
        Map
        Utility
        pthread
        )

if(YAML_IO)
    set(EIBA_LINK_LIBS ${EIBA_LINK_LIBS} ${YAML_CPP_LIBRARIES})
    add_executable(ExampleYAML Example/ExampleYAML.cpp)
    target_link_libraries(ExampleYAML ${EIBA_LINK_LIBS})
endif()
