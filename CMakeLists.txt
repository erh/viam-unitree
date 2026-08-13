cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(dds-unitree
    DESCRIPTION "DDS + Livox communication layer for Unitree G1"
    LANGUAGES C CXX
)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
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

# Livox-SDK2 (only sdk_core — skip samples). Used for Mid-360 when Unitree's
# DDS lidar publisher is not installed on the robot.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)
FetchContent_Declare(
    livox_sdk2
    GIT_REPOSITORY https://github.com/Livox-SDK/Livox-SDK2.git
    GIT_TAG v1.3.1
    GIT_SHALLOW TRUE
)
FetchContent_GetProperties(livox_sdk2)
if(NOT livox_sdk2_POPULATED)
    FetchContent_Populate(livox_sdk2)
    # sdk_core still declares cmake_minimum_required(VERSION 3.0); force policy floor.
    add_subdirectory(${livox_sdk2_SOURCE_DIR}/sdk_core ${livox_sdk2_BINARY_DIR}/sdk_core)
    # AppleClang treats some Livox SDK patterns as -Werror; relax so the vendor
    # sources build. Linux GCC builds are unaffected.
    foreach(_livox_tgt livox_lidar_sdk_static livox_lidar_sdk_shared)
        if(TARGET ${_livox_tgt})
            target_compile_options(${_livox_tgt} PRIVATE -Wno-error)
        endif()
    endforeach()
endif()

add_library(livox_mid360 STATIC capi/livox_mid360.cpp)
target_include_directories(livox_mid360
    PUBLIC ${CMAKE_SOURCE_DIR}/capi
    PRIVATE ${livox_sdk2_SOURCE_DIR}/include
)
target_link_libraries(livox_mid360 PUBLIC livox_lidar_sdk_static Threads::Threads)
