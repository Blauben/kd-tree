include(FetchContent)

message(STATUS "Setting up xsimd Library")
set(XSIMD_VERSION 11.1.0)
set(CMAKE_POLICY_VERSION_MINIMUM 3.10)

FetchContent_Declare(xsimd
        FIND_PACKAGE_ARGS ${XSIMD_VERSION} QUIET NAMES xsimd
        GIT_REPOSITORY https://github.com/xtensor-stack/xsimd.git
        GIT_TAG ${XSIMD_VERSION}
)

FetchContent_MakeAvailable(xsimd)

if (xsimd_FOUND)
    message(STATUS "Found existing xsimd Library: ${xsimd_DIR}")
else()
    message(STATUS "Using xsimd Library from GitHub Release ${XSIMD_VERSION}")
endif()
