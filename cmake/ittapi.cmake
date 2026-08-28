include(FetchContent)

message(STATUS "Setting up Intel ITT API")
set(ITTAPI_VERSION 3.28.0)

# Prefer an installed package when available, otherwise fetch the upstream
# oneAPI implementation so benchmark builds remain self-contained.
find_package(ittapi CONFIG QUIET)

if(ittapi_FOUND)
    message(STATUS "Found existing Intel ITT API package: ${ittapi_DIR}")
else()
    message(STATUS "Using Intel ITT API from GitHub Release ${ITTAPI_VERSION}")

    set(ITT_API_CPP_SUPPORT OFF CACHE BOOL "" FORCE)
    set(ITT_API_FORTRAN_SUPPORT OFF CACHE BOOL "" FORCE)
    set(ITT_API_IPT_SUPPORT OFF CACHE BOOL "" FORCE)
    set(ITT_API_REFERENCE_COLLECTOR OFF CACHE BOOL "" FORCE)
    set(ITT_API_INSTALL OFF CACHE BOOL "" FORCE)

    FetchContent_Declare(ittapi
            GIT_REPOSITORY https://github.com/intel/ittapi.git
            GIT_TAG v${ITTAPI_VERSION}
    )
    FetchContent_MakeAvailable(ittapi)
endif()
