include(FetchContent)

message(STATUS "Finding Python installation.")

find_package(Python 3.9 REQUIRED COMPONENTS Interpreter Development.Module)

# if (NOT PYTHON_FOUND)
#     message(STATUS "Python executable not found, the build might be incomplete! Please install Python.")
    # return()
# endif()

# message(STATUS "Python found at ${Python_EXECUTABLE}")

message(STATUS "Setting up Nanobind Library")

set(NANOBIND_VERSION 2.8.0)
find_package(nanobind QUIET)

if(${nanobind_FOUND})
    message(STATUS "Found existing Nanobind Library: ${nanobind_DIR}")
else()
    message(STATUS "Using Nanobind Library from GitHub Release ${NANOBIND_VERSION}")

    FetchContent_Declare(nanobind
            GIT_REPOSITORY https://github.com/wjakob/nanobind
            GIT_TAG v${NANOBIND_VERSION}
    )
    FetchContent_MakeAvailable(nanobind)
endif()