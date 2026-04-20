include(FetchContent)

FetchContent_Declare(indicators
        FIND_PACKAGE_ARGS QUIET NAMES indicators
        GIT_REPOSITORY https://github.com/p-ranav/indicators
)

FetchContent_MakeAvailable(indicators)

if (indicators_FOUND)
    message(STATUS "Found existing indicators Library: ${indicators_DIR}")
else()
    message(STATUS "Using indicators Library from GitHub")
endif ()
