include(FetchContent)

message(STATUS "Setting up thrust")
# NVIDIA/thrust was archived in favor of the NVIDIA/cccl monorepo (which still ships thrust's
# own CMakeLists.txt/find_package(Thrust) API unchanged under its "thrust" subdirectory), so it is fetched from there instead.

set(THRUST_VERSION 3.4.2)

find_package(Thrust ${THRUST_VERSION} QUIET)

if (${Thrust_FOUND})
    message(STATUS "Found existing thrust installation: ${Thrust_DIR}")
else()
    message(STATUS "Using thrust from GitHub Release ${THRUST_VERSION}")
    FetchContent_Declare(thrust
            GIT_REPOSITORY https://github.com/NVIDIA/cccl.git
            GIT_TAG v${THRUST_VERSION}
            GIT_SHALLOW TRUE
            )
    FetchContent_MakeAvailable(thrust)
endif()
