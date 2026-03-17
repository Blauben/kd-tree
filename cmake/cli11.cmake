include(FetchContent)

set(CLI11_VERSION 2.6.2)

# Try to find an existing installation
find_package(CLI11 ${CLI11_VERSION} QUIET)

if ($CLI11_FOUND})
    message(STATUS "Found existing CLI11: ${CLI11_DIR}")
else ()
    message(STATUS "Using CLI11 from GitHub Release ${CLI11_VERSION}")

    # Declare the source for Google Benchmark
    FetchContent_Declare(CLI11
            GIT_REPOSITORY https://github.com/CLIUtils/CLI11
            GIT_TAG v${CLI11_VERSION}
    )
    FetchContent_MakeAvailable(CLI11)
endif ()