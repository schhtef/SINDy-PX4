
add_executable(SINDy_offboard
    sid_control.cpp
    buffer.cpp
    system_identification.cpp
    regression.cpp
    interpolate.cpp
)

#set(CMAKE_PREFIX_PATH "/usr/aarch64-linux-gnu/local/lib/cmake/MAVSDK")
message(CMAKE_FIND_ROOT_PATH="${CMAKE_FIND_ROOT_PATH}")
#find_package(MAVSDK REQUIRED)
#find_package(Armadillo REQUIRED)

include_directories(${CMAKE_FIND_ROOT_PATH}/include)
include_directories(/usr/include)
link_directories(${CMAKE_FIND_ROOT_PATH}/lib)

#include_directories(${CMAKE_FIND_ROOT_PATH}/local/include)
#include_directories(${HDF5_INCLUDE_DIRS})
#if(SIL_BUILD)
#    link_directories(/usr/lib)
#elseif(HIL_BUILD)
#    link_directories(/usr/aarch64-linux-gnu)
#endif()
#set(HDF5_CXX_LIBRARIES /usr/lib/x86_64-linux-gnu/hdf5/serial)

target_link_libraries(SINDy_offboard
    mavsdk
    pthread
    #mavsdk
    armadillo
    #openblas
    #lapack
)