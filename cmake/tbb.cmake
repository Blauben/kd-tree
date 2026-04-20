include(FetchContent)

message(STATUS "Setting up tbb")
set(TBB_VERSION 2021.12.0)

# Skip find_package on MSVC — pip/conda TBB packages ship MinGW-format
# .dll.a import libs which MSVC's linker cannot use.
if(NOT MSVC)
    FetchContent_Declare(tbb
            FIND_PACKAGE_ARGS QUIET NAMES TBB
            GIT_REPOSITORY https://github.com/oneapi-src/oneTBB.git
            GIT_TAG v${TBB_VERSION}
    )
else()
    FetchContent_Declare(tbb
            GIT_REPOSITORY https://github.com/oneapi-src/oneTBB.git
            GIT_TAG v${TBB_VERSION}
    )
endif()

FetchContent_MakeAvailable(tbb)

if (TBB_FOUND)
    message(STATUS "Found existing TBB library: ${TBB_DIR}")
else()
    message(STATUS "Using TBB from GitHub Release ${TBB_VERSION}")

    # Disable tests & and do not treat tbb-compile errors as warnings
    set(TBB_TEST OFF CACHE BOOL "" FORCE)
    set(TBB_STRICT OFF CACHE STRING "" FORCE)
endif()

# Copies runtime DLLs (primarily libtbb12.dll) next to a target on Windows
# so executables and modules run directly from the build folder without PATH changes.
function(kd_tree_copy_runtime_dlls target_name)
    if (NOT WIN32)
        return()
    endif ()

    if (CMAKE_VERSION VERSION_GREATER_EQUAL "3.21")
        add_custom_command(TARGET ${target_name} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_RUNTIME_DLLS:${target_name}>
                $<TARGET_FILE_DIR:${target_name}>
                COMMAND_EXPAND_LISTS
                VERBATIM
        )
    elseif (TARGET TBB::tbb)
        # Fallback for older CMake versions where TARGET_RUNTIME_DLLS is unavailable.
        add_custom_command(TARGET ${target_name} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:TBB::tbb>
                $<TARGET_FILE_DIR:${target_name}>
                VERBATIM
        )
    endif ()
endfunction()

