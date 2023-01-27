cmake_minimum_required(VERSION 3.10.2)

option(SIL_BUILD_TEST "Build the test suite" OFF)
option(SIL_BUILD "Build the application for SIL environment on dev machine" OFF)
option(HIL_BUILD "Build the application for HIL environment on RPI" OFF)

#set toolchain file for embedded system
#if(HIL_BUILD)
#    set(CMAKE_TOOLCHAIN_FILE Toolchain-aarch64-linux-gnu.cmake)
#endif()

project(SINDy_offboard)

#set(CMAKE_VERBOSE_MAKEFILE ON)
#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CXX_FLAGS_DEBUG  "-g")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#add_definitions("-Wall -Wextra -Werror")
add_definitions("-std=c++17")

include_directories(/usr/include/mavsdk)
include_directories(/src)
link_directories(/usr/lib)

#Build the executable
if(SIL_BUILD OR HIL_BUILD)
    add_subdirectory(src)
endif()

#Build unit tests
if (SIL_BUILD_TEST)
    include(CTest)
    add_subdirectory(test)
endif()