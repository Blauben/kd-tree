include(FetchContent)

message(STATUS "Setting up thrust")
# NVIDIA/thrust was archived in favor of the NVIDIA/cccl monorepo (which still ships thrust's
# own CMakeLists.txt/find_package(Thrust) API unchanged under its "thrust" subdirectory), so it is fetched from there instead.
# Switching to cccl drops THRUST_ENABLE_HEADER_TESTING/THRUST_ENABLE_TESTING/THRUST_ENABLE_EXAMPLES/THRUST_CPP_DIALECT options

set(THRUST_VERSION 3.4.2)

# Parent project (e.g. via FetchContent) already set thrust up -> reuse it, otherwise both
# end up fighting over the same "thrust" FetchContent name / _deps/thrust-src directory
if (COMMAND thrust_create_target)
    message(STATUS "Thrust already provided by the parent project. Using existing setup.")
    return()
endif ()

find_package(Thrust ${THRUST_VERSION} QUIET)

if (${Thrust_FOUND})
    message(STATUS "Found existing thrust installation: ${Thrust_DIR}")
else()
    message(STATUS "Using thrust from GitHub Release ${THRUST_VERSION}")
    FetchContent_Declare(thrust
            GIT_REPOSITORY https://github.com/NVIDIA/cccl.git
            GIT_TAG v${THRUST_VERSION}
            GIT_SHALLOW TRUE
            # cccl has paths way past MAX_PATH, clone dies on Windows without this
            GIT_CONFIG core.longpaths=true
            )
    FetchContent_MakeAvailable(thrust)
endif()
