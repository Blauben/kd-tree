include(FetchContent)

message(STATUS "Setting up Google Benchmark")
set(GOOGLE_BENCHMARK_VERSION 1.9.4)

set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "Disable benchmark testing" FORCE)
include(gtest)

FetchContent_Declare(googlebenchmark
        FIND_PACKAGE_ARGS ${GOOGLE_BENCHMARK_VERSION} QUIET NAMES benchmark
        GIT_REPOSITORY https://github.com/google/benchmark.git
        GIT_TAG v${GOOGLE_BENCHMARK_VERSION}
)

FetchContent_MakeAvailable(googlebenchmark)

if (benchmark_FOUND)
    message(STATUS "Found existing Google Benchmark: ${benchmark_DIR}")
else ()
    message(STATUS "Using Google Benchmark from GitHub Release ${GOOGLE_BENCHMARK_VERSION}")

    # Retrieve and propagate include directories for Google Benchmark
    get_target_property(propval benchmark INTERFACE_INCLUDE_DIRECTORIES)
    target_include_directories(benchmark SYSTEM PUBLIC "${propval}")
endif ()
