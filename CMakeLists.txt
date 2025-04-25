cmake_minimum_required(VERSION 3.16)
project(kd-tree)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Appends the the module path to contain additional CMake modules for this project
# and include everything necessary
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(CMakeDependentOption)

#####################################
# PARALLELIZATION OPTIONS AND LOGGING
#####################################
# Parallelization on the HOST
set(KDTREE_PARALLELIZATION "CPP" CACHE STRING "Host parallelization chosen by the user
 (CPP= Serial, TBB = Intel Threading Building Blocks")
set_property(CACHE KDTREE_PARALLELIZATION PROPERTY STRINGS CPP, OMP, TBB)

#########################################################
# What actually to build? - Options, Versions and Output
#########################################################
# Option to enable Include-What-You-Use warnings during compilation
option(BUILD_KDTREE_LIBRARY "Set to on to build the KD-Tree Library (Default: ON)" ON)
option(BUILD_KDTREE_EXECUTABLE "Set to on to build the KD-Tree Executable (Default: ON)" ON)
option(ENABLE_IWYU "Set to on to enable Include-What-You-Use warnings (Default: OFF)" OFF)

IF (_LIBCPP_DISABLE_AVAILABILITY)
    message(STATUS "Disabling availability macros for libc++")
    add_definitions(-D_LIBCPP_DISABLE_AVAILABILITY)
endif ()

# Resolves missing fmt symbols when working with spdlog (bundled via brew/ conda on Arm architecture)
# Refer to https://github.com/gabime/spdlog/issues/660
add_compile_definitions(FMT_HEADER_ONLY)

include(git)
include(version.cmake)

message(STATUS "#################################################################")
message(STATUS "KD Tree Version          ${KDTREE_VERSION}")
message(STATUS "KD Tree Commit Hash      ${KDTREE_COMMIT_HASH}")
message(STATUS "KD Tree Parallelization Backend  ${KDTREE_PARALLELIZATION}")
message(STATUS "#################################################################")
#######################################################
# Including dependencies needed across multiple targets
#######################################################

include(thrust)
include(spdlog)
include(tetgen)
include(xsimd)
include(clang_format)

if (${ENABLE_IWYU})
    include(include-what-you-use)
endif ()

###############################
# Thrust Parallelization Set-Up
###############################
# Get a version of tbb from the github repository, simplifies compilation for the user since tbb does not need to be
# preinstalled but rather gets automatically set up via CMake
# Nevertheless, there is still the option to enforce to use a local installation if one exists
if (${KDTREE_PARALLELIZATION} STREQUAL "TBB")
    include(tbb)
    thrust_set_TBB_target(TBB::tbb)
    add_compile_definitions(KDTREE_TBB)
endif ()

# Thrust set-up i.e. the parallelization library, create targets according to the users specification
thrust_create_target(Thrust HOST CPP DEVICE ${KDTREE_PARALLELIZATION})
message(STATUS "Set Parallelization: HOST CPP DEVICE ${KDTREE_PARALLELIZATION}")

############################################################
# Polyhedral Gravity Library & Executable & Python Interface
############################################################
add_subdirectory(${PROJECT_SOURCE_DIR}/src)