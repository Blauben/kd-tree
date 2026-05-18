include(FetchContent)

find_package(indicators QUIET)

if (${indicators_FOUND})
    message(STATUS "Found existing indicators Library: ${indicators_DIR}")
else()
    message(STATUS "Using indicators Library from GitHub")
    FetchContent_Declare(
        indicators
        GIT_REPOSITORY https://github.com/p-ranav/indicators
    )
    FetchContent_MakeAvailable(indicators)
endif ()
