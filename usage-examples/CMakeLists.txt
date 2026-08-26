# Standalone CMake project for the gravity particle simulation usage example.
# Configure and build independently of the main KDTree project, e.g.:
#   cmake -S usage-examples -B usage-examples/build
#   cmake --build usage-examples/build
cmake_minimum_required(VERSION 3.16)
project(KDTreeGravityDemo CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build" FORCE)
endif ()

# Pull in the KDTree library from the parent repository as a subdirectory. Only the
# library itself is needed here, so the KDTree project's own executable/tests/python
# interface stay off (their defaults already do this for a non-top-level build).
# Quieten the library's own logging so it doesn't drown out this example's per-step
# output, since the tree is rebuilt every simulation step.
set(KD_TREE_LOGGING_LEVEL "WARN" CACHE STRING "Logging level for the KDTree library")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_BINARY_DIR}/kdtree-lib)

# The parent project's own CMake module path (which provides the cli11 module below) is
# scoped to its own add_subdirectory() call above and doesn't propagate back up here, so
# it needs to be added again for this project's own scope.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)
include(cli11)

add_executable(gravity_sim
        src/main.cpp
        src/Simulation.cpp
)
target_link_libraries(gravity_sim PRIVATE KDTree::KDTree_lib CLI11::CLI11)
