Include(FetchContent)

FetchContent_Declare(
  TOML11
  GIT_REPOSITORY https://github.com/ToruNiina/toml11
  GIT_TAG        v3.7.1 # or a later release
)

FetchContent_MakeAvailable(TOML11)

include_directories(${PROJECT_SOURCE_DIR}/thirdparty/toml11)

add_executable(SINDy_offboard
    sid_control.cpp
    buffer.cpp
    system_identification.cpp
    regression.cpp
    interpolate.cpp
    logging.cpp
)


find_package(MAVSDK REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(${ARMADILLO_INCLUDE_DIRS})

target_link_libraries(SINDy_offboard
    MAVSDK::mavsdk
    pthread
    armadillo
)