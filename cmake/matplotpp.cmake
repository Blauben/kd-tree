include(FetchContent)

set(MATPLOTPP_VERSION 1.2.0)

# Try an existing CMake package first (official package name).
find_package(Matplot++ CONFIG QUIET)

if (Matplot++_FOUND)
    message(STATUS "Found existing matplot++ library: ${Matplot++_DIR}")
    if (TARGET Matplot++::matplot AND NOT TARGET matplot)
        add_library(matplot ALIAS Matplot++::matplot)
    endif ()
else()
    message(STATUS "Using matplot++ from GitHub Release ${MATPLOTPP_VERSION}")

    # Keep the fetched dependency lean when embedded in this project.
    set(MATPLOTPP_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(MATPLOTPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(MATPLOTPP_BUILD_INSTALLER OFF CACHE BOOL "" FORCE)
    # Avoid third-party warning policies turning warnings into hard errors on MSVC.
    set(MATPLOTPP_BUILD_WITH_PEDANTIC_WARNINGS OFF CACHE BOOL "" FORCE)
    set(MATPLOTPP_WITH_OPENCV OFF CACHE BOOL "" FORCE)
    set(MATPLOTPP_WITH_SYSTEM_CIMG OFF CACHE BOOL "" FORCE)
    set(MATPLOTPP_WITH_SYSTEM_NODESOUP OFF CACHE BOOL "" FORCE)

    # On MSVC, avoid auto-detecting MinGW/MSYS packages that inject incompatible headers/libs.
    if (MSVC)
        set(CMAKE_DISABLE_FIND_PACKAGE_PkgConfig ON CACHE BOOL "" FORCE)
        set(CMAKE_DISABLE_FIND_PACKAGE_JPEG ON CACHE BOOL "" FORCE)
        set(CMAKE_DISABLE_FIND_PACKAGE_TIFF ON CACHE BOOL "" FORCE)
        set(CMAKE_DISABLE_FIND_PACKAGE_ZLIB ON CACHE BOOL "" FORCE)
        set(CMAKE_DISABLE_FIND_PACKAGE_PNG ON CACHE BOOL "" FORCE)
        set(CMAKE_DISABLE_FIND_PACKAGE_LAPACK ON CACHE BOOL "" FORCE)
        set(CMAKE_DISABLE_FIND_PACKAGE_BLAS ON CACHE BOOL "" FORCE)
        set(CMAKE_DISABLE_FIND_PACKAGE_FFTW ON CACHE BOOL "" FORCE)
        set(CMAKE_DISABLE_FIND_PACKAGE_OpenCV ON CACHE BOOL "" FORCE)
    endif ()

    FetchContent_Declare(matplotplusplus
        GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
        GIT_TAG v${MATPLOTPP_VERSION})
    FetchContent_MakeAvailable(matplotplusplus)

    # Keep strict warnings for this project, but do not fail on third-party warnings.
    if (TARGET matplot)
        set_target_properties(matplot PROPERTIES COMPILE_WARNING_AS_ERROR OFF)
        if (MSVC)
            target_compile_options(matplot PRIVATE /wd4834)
        endif ()
    endif ()
endif ()