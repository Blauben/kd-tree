message(STATUS "Setting up gtest")

include(FetchContent)

message(STATUS "Setting up Google Benchmark")
set(GOOGLE_TEST_VERSION 1.17.0)

if (KD_TREE_FORCE_GTEST_FETCH)
    FetchContent_Declare(googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG v${GOOGLE_TEST_VERSION}
    )
else ()
    FetchContent_Declare(googletest
            FIND_PACKAGE_ARGS ${GOOGLE_TEST_VERSION} QUIET NAMES GTest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG v${GOOGLE_TEST_VERSION}
    )
endif ()

FetchContent_MakeAvailable(googletest)

if (GTest_FOUND)
    message(STATUS "Found existing Google Test: ${GTEST_INCLUDE_DIRS}")
else ()
    message(STATUS "Using Google Test from GitHub Release ${GOOGLE_TEST_VERSION}")
    target_compile_options(gtest PRIVATE -w)
    target_compile_options(gtest_main PRIVATE -w)
    target_compile_options(gmock PRIVATE -w)
    target_compile_options(gmock_main PRIVATE -w)

    get_target_property(propval gtest_main INTERFACE_INCLUDE_DIRECTORIES)
    target_include_directories(gtest_main SYSTEM PUBLIC "${propval}")
endif ()