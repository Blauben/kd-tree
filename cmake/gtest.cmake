include(FetchContent)

message(STATUS "Setting up Google Test")
set(GOOGLE_TEST_VERSION 1.15.2)

# Don't search for system GTest at all - always fetch
set(BUILD_GMOCK ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)

message(STATUS "Fetching Google Test from GitHub Release ${GOOGLE_TEST_VERSION}")

FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v${GOOGLE_TEST_VERSION}
)
FetchContent_MakeAvailable(googletest)

# Suppress warnings from gtest
target_compile_options(gtest PRIVATE -w)
target_compile_options(gtest_main PRIVATE -w)
target_compile_options(gmock PRIVATE -w)
target_compile_options(gmock_main PRIVATE -w)

# Make include directories system to suppress warnings
get_target_property(propval gtest_main INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(gtest_main SYSTEM PUBLIC "${propval}")