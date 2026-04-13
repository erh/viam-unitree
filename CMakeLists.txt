cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(unitree-capi
    DESCRIPTION "C wrapper around unitree_sdk2 for CGo"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include(FetchContent)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    unitree_sdk2
    GIT_REPOSITORY https://github.com/unitreerobotics/unitree_sdk2.git
    GIT_TAG 2.0.2
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(unitree_sdk2)

add_library(unitree_capi STATIC capi/unitree_capi.cpp)
target_include_directories(unitree_capi PUBLIC ${CMAKE_SOURCE_DIR}/capi)
target_link_libraries(unitree_capi PUBLIC unitree_sdk2)
