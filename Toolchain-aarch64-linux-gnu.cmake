# Define our host system
# If you set CMAKE_SYSTEM_NAME manually, CMake will automatically set CMAKE_CROSSCOMPILING to TRUE 
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_SYSTEM_VERSION 1)

set(target_arch aarch64-linux-gnu)

# Define the cross compiler locations
SET(CMAKE_C_COMPILER   /usr/bin/aarch64-linux-gnu-gcc)
SET(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)

# Define the path for which linked libraries should be found
# We want to search in this folder since the linux libraries like pthread
# Have been precompiled for arm here, rather than in the default system folder 
# We also store cross-compiled versions of linked libraries here, namely armadillo
set(CMAKE_FIND_ROOT_PATH "/usr/${target_arch}")

# Adjust the default behavior of the FIND_XXX() commands:
# search programs in the host environment only.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# Search for libraries and headers in the arm-linux directories only
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

 # specify the toolchain programs
 #find_program(CMAKE_C_COMPILER ${target_arch}-gcc)
 #find_program(CMAKE_CXX_COMPILER ${target_arch}-g++)
 if(NOT CMAKE_C_COMPILER OR NOT CMAKE_CXX_COMPILER)
     message(FATAL_ERROR "Can't find suitable C/C++ cross compiler for ${target_arch}")
 endif()

 set(CMAKE_AR ${target_arch}-ar CACHE FILEPATH "" FORCE)
 set(CMAKE_RANLIB ${target_arch}-ranlib)
 set(CMAKE_LINKER ${target_arch}-ld)