include(FetchContent)

message(STATUS "Setting up Nanobind Library")

set(NANOBIND_VERSION 2.8.0)
find_package(nanobind QUIET)

if(${nanobind_FOUND})
    message(STATUS "Found existing Nanobind Library: ${nanobind_DIR}")
else()
    message(STATUS "Using Nanobind Library from GitHub Release ${NANOBIND_VERSION}")

    FetchContent_Declare(nanobind
            GIT_REPOSITORY https://github.com/wjakob/nanobind
            GIT_TAG v${NANOBIND_VERSION}
    )
    FetchContent_MakeAvailable(nanobind)
endif()