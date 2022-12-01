#Get the most recent Catch2 library
Include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.0.1 # or a later release
)

FetchContent_MakeAvailable(Catch2)

find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/src)

add_executable(SINDy_tests
    tests.cpp
    ${PROJECT_SOURCE_DIR}/src/regression.cpp
    ${PROJECT_SOURCE_DIR}/src/system_identification.cpp
)

#Link the required libraries, including the Catch2 with Main library
#Catch2WithMain calls its own main(), so the test.cpp file does not need a main()
target_link_libraries(SINDy_tests
    PRIVATE
    Catch2::Catch2WithMain
    armadillo
    MAVSDK::mavsdk
    pthread
)

#Add the test
enable_testing()
add_test(NAME test COMMAND SINDy_tests)