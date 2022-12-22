
add_executable(SINDy_offboard
    sid_control.cpp
    buffer.cpp
    system_identification.cpp
    regression.cpp
    interpolate.cpp
)

find_package(MAVSDK REQUIRED)
#find_package(Armadillo REQUIRED)

include_directories(/usr/include)
#include_directories(${HDF5_INCLUDE_DIRS})
#if(SIL_BUILD)
#    link_directories(/usr/lib)
#elseif(HIL_BUILD)
#    link_directories(/usr/aarch64-linux-gnu)
#endif()
#set(HDF5_CXX_LIBRARIES /usr/lib/x86_64-linux-gnu/hdf5/serial)

target_link_libraries(SINDy_offboard
    MAVSDK::mavsdk
    pthread
    #armadillo
    openblas
)