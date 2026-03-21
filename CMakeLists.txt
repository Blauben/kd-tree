# The CMake config files and structure were adapted from:
#   https://https://github.com/esa/polyhedral-gravity-model
#
# Original authors: Schuhmacher, J., Blazquez, E., Gratl, F., Izzo, D., & Gómez, P.
# License: GPL-3.0

cmake_minimum_required(VERSION 3.16)
project(KDTree)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(KD_TREE_SOURCE_DIR ${PROJECT_SOURCE_DIR})
set(KD_TREE_BINARY_DIR ${PROJECT_BINARY_DIR})

# Appends the the module path to contain additional CMake modules for this project
# and include everything necessary
list(APPEND CMAKE_MODULE_PATH ${KD_TREE_SOURCE_DIR}/cmake)
include(CMakeDependentOption)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build (Debug, Release, MinSizeRel, or RelWithDebInfo)" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()
message(STATUS "CMAKE_BUILD_TYPE set to ${CMAKE_BUILD_TYPE}")

#####################################
# PARALLELIZATION OPTIONS AND LOGGING
#####################################
# Parallelization on the HOST
set(KD_TREE_PARALLELIZATION "CPP" CACHE STRING "Host parallelization chosen by the user
 (CPP= Serial, OMP = OpenMP, TBB = Intel Threading Building Blocks")
set_property(CACHE KD_TREE_PARALLELIZATION PROPERTY STRINGS CPP OMP TBB)

# Set the Logging Level
set(KD_TREE_LOGGING_LEVEL_LIST "TRACE" "DEBUG" "INFO" "WARN" "ERROR" "CRITICAL" "OFF")
set(KD_TREE_LOGGING_LEVEL "INFO" CACHE STRING "Set the Logging level, default (INFO), available options: TRACE, DEBUG, INFO, WARN, ERROR, CRITICAL, OFF")
set_property(CACHE KD_TREE_LOGGING_LEVEL PROPERTY STRINGS ${KD_TREE_LOGGING_LEVEL_LIST})
# Convert the logging level string to its corresponding number
list(FIND KD_TREE_LOGGING_LEVEL_LIST ${KD_TREE_LOGGING_LEVEL} LOGGING_LEVEL_INDEX)
if (${LOGGING_LEVEL_INDEX} EQUAL -1)
    message(FATAL_ERROR "Invalid logging level: ${KD_TREE_LOGGING_LEVEL}")
endif ()
add_compile_definitions(SPDLOG_ACTIVE_LEVEL=${LOGGING_LEVEL_INDEX})

#########################################################
# What actually to build? - Options, Versions and Output
#########################################################
# Build docs
option(BUILD_KD_TREE_DOCS "Builds the documentation (Default: OFF)" OFF)
# Build C++ executable
option(BUILD_KD_TREE_EXECUTABLE "Builds the C++ executable (Default: ON)" ON)
# Build library (default ON), if the executable or tests are built this forced to ON
cmake_dependent_option(BUILD_KD_TREE_LIBRARY "Builds the library (Default: ON)" ON
        "NOT BUILD_KD_TREE_EXECUTABLE AND NOT BUILD_KD_TREE_TESTS" ON)
# Option to build the python interface
option(BUILD_KD_TREE_PYTHON_INTERFACE "Set this to on if the python interface should be built (Default: OFF)" OFF)
# Option to install the python interface
option(INSTALL_KD_TREE_PYTHON_INTERFACE "Set this to on if the python interface should be installed (Default: OFF)" OFF)
# Option to build tests or not
option(BUILD_KD_TREE_TESTS "Set to on if the tests should be built (Default: OFF)" OFF)
# Option to force fetching Google Test even if it is already installed locally
option(KD_TREE_FORCE_GTEST_FETCH "Force the the fetching of Google Test (Default: OFF)" OFF)
# Option to build benchmarks or not
option(BUILD_KD_TREE_TIME_MEASUREMENT "Set to on if the benchmark executable should be built (Default: OFF)" OFF)
# Option to enable Include-What-You-Use warnings during compilation
option(ENABLE_IWYU "Set to on to enable Include-What-You-Use warnings (Default: OFF)" OFF)

IF (_LIBCPP_DISABLE_AVAILABILITY)
    message(STATUS "Disabling availability macros for libc++")
    add_definitions(-D_LIBCPP_DISABLE_AVAILABILITY)
endif ()

# AppleClang options
if (${CMAKE_SYSTEM_NAME} STREQUAL "Darwin" AND (${CMAKE_CXX_COMPILER_ID} STREQUAL "AppleClang" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang"))  
    # Fixes undefined _VSTD when using Apple Clang 17  
    message(STATUS "Using Apple Clang compiler. Defining _VSTD=std.")  
    add_definitions(-D_VSTD=std)  
endif()


# Resolves missing fmt symbols when working with spdlog (bundled via brew/ conda on Arm architecture)
# Refer to https://github.com/gabime/spdlog/issues/660
add_compile_definitions(FMT_HEADER_ONLY)

include(git)
include(version.cmake)

message(STATUS "#################################################################")
message(STATUS "KD Tree Version          ${KD_TREE_VERSION}")
message(STATUS "KD Tree Commit Hash      ${KD_TREE_COMMIT_HASH}")
message(STATUS "KD Tree Parallelization Backend  ${KD_TREE_PARALLELIZATION}")
message(STATUS "KD Tree Logging Level    ${KD_TREE_LOGGING_LEVEL}")
message(STATUS "#################################################################")
message(STATUS "KD Tree Documentation    ${BUILD_KD_TREE_DOCS}")
message(STATUS "KD Tree Library          ${BUILD_KD_TREE_LIBRARY}")
message(STATUS "KD Tree C++ Executable   ${BUILD_KD_TREE_EXECUTABLE}")
message(STATUS "KD Tree Python Interface ${BUILD_KD_TREE_PYTHON_INTERFACE}")
message(STATUS "KD Tree Tests            ${BUILD_KD_TREE_TESTS}")
message(STATUS "KD Tree Time Benchmark   ${BUILD_KD_TREE_TIME_MEASUREMENT}")
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
if (${KD_TREE_PARALLELIZATION} STREQUAL "TBB")
    include(tbb)
    thrust_set_TBB_target(TBB::tbb)
    add_compile_definitions(KD_TREE_TBB)
elseif (${KD_TREE_PARALLELIZATION} STREQUAL "OMP")
    add_compile_definitions(KD_TREE_OMP)
endif ()

# Thrust set-up i.e. the parallelization library, create targets according to the users specification
thrust_create_target(Thrust HOST CPP DEVICE ${KD_TREE_PARALLELIZATION})
message(STATUS "Set Parallelization: HOST CPP DEVICE ${KD_TREE_PARALLELIZATION}")

# Fallback for non-TBB builds: tbb.cmake defines the real implementation.
if (NOT COMMAND kd_tree_copy_runtime_dlls)
    function(kd_tree_copy_runtime_dlls target_name)
    endfunction()
endif ()


############################################################
# KD Tree Library & Executable & Python Interface
############################################################
add_subdirectory(${KD_TREE_SOURCE_DIR}/src)

##############################
# Building the Polyhedral Docs
##############################
if (BUILD_KD_TREE_DOCS)
    add_subdirectory(${KD_TREE_SOURCE_DIR}/docs)
endif ()

###############################
# Building the Polyhedral Tests
###############################
if (BUILD_KD_TREE_TESTS)
    message(STATUS "Building the KD Tree Tests")
    # Enables CTest, must be in the top-level CMakeList.txt, otherwise it won't work
    enable_testing()

    # Subdirectory where the tests are located
    add_subdirectory(${KD_TREE_SOURCE_DIR}/test)
endif ()