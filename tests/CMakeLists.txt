cmake_minimum_required(VERSION 3.10.2)

#set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG  "-g")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(SINDy_tests)

#add_definitions("-Wall -Wextra -Werror")
add_definitions("-std=c++17")

include_directories(/usr/include/mavsdk)
link_directories(/usr/lib)

add_executable(SINDy_tests
    unit_tests.cpp
    regression.cpp
)

find_package(MAVSDK REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(${ARMADILLO_INCLUDE_DIRS})

target_link_libraries(SINDy_tests
    MAVSDK::mavsdk
    armadillo
)
