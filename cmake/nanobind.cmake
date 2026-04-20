include(FetchContent)

message(STATUS "Finding Python installation.")

find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)

message(STATUS "Setting up Nanobind Library")

set(NANOBIND_VERSION 2.8.0)

FetchContent_Declare(nanobind
        FIND_PACKAGE_ARGS QUIET NAMES nanobind
        GIT_REPOSITORY https://github.com/wjakob/nanobind
        GIT_TAG v${NANOBIND_VERSION}
)

FetchContent_MakeAvailable(nanobind)

if (nanobind_FOUND)
    message(STATUS "Found existing Nanobind Library: ${nanobind_DIR}")
else()
    message(STATUS "Using Nanobind Library from GitHub Release ${NANOBIND_VERSION}")
endif()