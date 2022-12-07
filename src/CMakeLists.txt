
add_executable(SINDy_offboard
    sid_control.cpp
    buffer.cpp
    system_identification.cpp
    regression.cpp
    interpolate.cpp
)

find_package(MAVSDK REQUIRED)
find_package(Armadillo REQUIRED)
find_package(HDF5 REQUIRED)

include_directories(${ARMADILLO_INCLUDE_DIRS})
include_directories(${HDF5_INCLUDE_DIRS})
link_directories(/usr/lib)

set(HDF5_CXX_LIBRARIES /usr/lib/x86_64-linux-gnu/hdf5/serial)

target_link_libraries(SINDy_offboard
    MAVSDK::mavsdk
    pthread
    armadillo
    hdf5::hdf5_cpp
)