cmake_minimum_required(VERSION 3.10.2)

option(SIL_BUILD_TEST "Build the test suite" OFF)
option(SIL_BUILD "Build the application for SIL environment on dev machine" ON)

project(SINDy_offboard)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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