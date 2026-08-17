include(FetchContent)

message(STATUS "Setting up thrust")
# NVIDIA/thrust was archived in favor of the NVIDIA/cccl monorepo (which still ships thrust's
# own CMakeLists.txt/find_package(Thrust) API unchanged under its "thrust" subdirectory), so it
# is fetched from there instead. The last standalone thrust release (2.1.0) bundles an old
# libcudacxx whose unprefixed "_NOEXCEPT_" macro collides with recent Apple libc++ headers on
# macOS; cccl's libcudacxx renamed its internal macros (e.g. to "_CCCL_*") to avoid exactly this
# kind of collision with system headers.
set(THRUST_VERSION 3.4.2)

# Set custom variables, policies, etc.
# Disable stuff not needed
set(THRUST_ENABLE_HEADER_TESTING "OFF")
set(THRUST_ENABLE_TESTING "OFF")
set(THRUST_ENABLE_EXAMPLES "OFF")
# Set standard CPP Dialect to 17 (default of thrust would be 14)
set(THRUST_CPP_DIALECT 17)

find_package(Thrust ${THRUST_VERSION} QUIET)


if (${Thrust_FOUND})
    message(STATUS "Found existing thrust installation: ${Thrust_DIR}")
else()
    message(STATUS "Using thrust from GitHub Release ${THRUST_VERSION}")
    FetchContent_Declare(thrust
            GIT_REPOSITORY https://github.com/NVIDIA/cccl.git
            GIT_TAG v${THRUST_VERSION}
            )
    FetchContent_MakeAvailable(thrust)
endif()
