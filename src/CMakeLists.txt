cmake_minimum_required(VERSION 3.10.2)

#set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(SINDy_offboard)

#add_definitions("-Wall -Wextra -Werror")
add_definitions("-std=c++2a")

include_directories(/usr/include/mavsdk)
link_directories(/usr/lib)

add_executable(SINDy_offboard
    sid_control.cpp
    buffer.cpp
    system_identification.cpp
    regression.cpp
)

find_package(MAVSDK REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(${ARMADILLO_INCLUDE_DIRS})

target_link_libraries(SINDy_offboard
    MAVSDK::mavsdk
    armadillo
)