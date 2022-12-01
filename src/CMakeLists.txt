
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
    pthread
    armadillo
)