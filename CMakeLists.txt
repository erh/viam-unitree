cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(dds-unitree
    DESCRIPTION "DDS communication layer for Unitree robots (pure CycloneDDS, no unitree SDK)"
    LANGUAGES C
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include(FetchContent)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_IDLC OFF CACHE BOOL "" FORCE)
set(BUILD_DDSPERF OFF CACHE BOOL "" FORCE)
set(ENABLE_SSL OFF CACHE BOOL "" FORCE)
set(ENABLE_SECURITY OFF CACHE BOOL "" FORCE)
set(ENABLE_SHM OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    cyclonedds
    GIT_REPOSITORY https://github.com/eclipse-cyclonedds/cyclonedds.git
    GIT_TAG 0.10.2
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(cyclonedds)

add_library(dds_unitree STATIC capi/dds_unitree.c)
target_include_directories(dds_unitree PUBLIC ${CMAKE_SOURCE_DIR}/capi)
target_link_libraries(dds_unitree PUBLIC ddsc)
