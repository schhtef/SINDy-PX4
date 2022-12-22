# Define our host system
# If you set CMAKE_SYSTEM_NAME manually, CMake will automatically set CMAKE_CROSSCOMPILING to TRUE 
SET(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)
SET(CMAKE_SYSTEM_VERSION 1)

# Define the cross compiler locations
SET(CMAKE_C_COMPILER   /usr/bin/aarch64-linux-gnu-gcc)
SET(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)

# Define the path for which linked libraries should be found
# We want to search in this folder since the linux libraries like pthread
# Have been precompiled for arm here, rather than in the default system folder 
# We also store cross-compiled versions of linked libraries here, namely armadillo
SET(CMAKE_FIND_ROOT_PATH /usr/aarch64-linux-gnu/)

# Adjust the default behavior of the FIND_XXX() commands:
# search programs in the host environment only.
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# Search for libraries and headers in the arm-linux directories only
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)