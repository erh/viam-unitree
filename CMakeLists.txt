cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(viam-unitree
    DESCRIPTION "Viam Unitree G1 Module"
    LANGUAGES CXX
)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(viam-cpp-sdk CONFIG REQUIRED COMPONENTS viamsdk)

include(FetchContent)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    unitree_sdk2
    GIT_REPOSITORY https://github.com/unitreerobotics/unitree_sdk2.git
    GIT_TAG 2.0.2
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(unitree_sdk2)

add_executable(viam-unitree
    main.cpp
    src/g1_base.cpp
    src/g1_camera.cpp
)

target_include_directories(viam-unitree PUBLIC src)

target_link_libraries(viam-unitree
    viam-cpp-sdk::viamsdk
    unitree_sdk2
)

install(
    FILES meta.json
    DESTINATION .
)

install(TARGETS viam-unitree)

set(CPACK_PACKAGE_NAME "viam-unitree")
set(CPACK_PACKAGE_FILE_NAME "module")
set(CPACK_GENERATOR "TGZ")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
include(CPack)
