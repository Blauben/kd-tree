include(FetchContent)

message(STATUS "Setting up spdlog")
set(SPDLOG_VERSION 1.17.0)

# Known Issue:
# If you install spdlog@1.14.1 via homebrew on ARM macOS, CMake will find spdlog
# However, there is a version mismatch between the `fmt` library installed as dependency, and the one actually
# being required leading to a linking error (i.e. missing symbols) while compiling!
# Update 29.11.2024: We fixed this by using spdlog has header library (--> top-level CMake file)

# Only set the active logging level if the parent project has not already defined it - overriding it here would
# silently discard a level the parent explicitly chose.
get_directory_property(_kd_tree_compile_defs COMPILE_DEFINITIONS)
set(_kd_tree_spdlog_level_predefined FALSE)
foreach(_kd_tree_def ${_kd_tree_compile_defs})
    if(_kd_tree_def MATCHES "^SPDLOG_ACTIVE_LEVEL=")
        set(_kd_tree_spdlog_level_predefined TRUE)
    endif()
endforeach()

if(_kd_tree_spdlog_level_predefined)
    message(WARNING "SPDLOG_ACTIVE_LEVEL is already defined by the parent project - KD_TREE_LOGGING_LEVEL (${KD_TREE_LOGGING_LEVEL}) will be ignored.")
else()
    # Convert the logging level string to its corresponding number
    list(FIND KD_TREE_LOGGING_LEVEL_LIST ${KD_TREE_LOGGING_LEVEL} LOGGING_LEVEL_INDEX)
    if (${LOGGING_LEVEL_INDEX} EQUAL -1)
        message(FATAL_ERROR "Invalid logging level: ${KD_TREE_LOGGING_LEVEL}")
    endif ()
    # Add the logging level index as a compile definition for the build
    add_compile_definitions(SPDLOG_ACTIVE_LEVEL=${LOGGING_LEVEL_INDEX})
endif()

if(TARGET spdlog::spdlog)
    message(STATUS "Found existing spdlog Library target: spdlog::spdlog")
    return()
endif()

if(${spdlog_FOUND})
    message(STATUS "Found existing spdlog Library: ${spdlog_DIR}")
else()
    message(STATUS "Using Spdlog Library from GitHub Release ${SPDLOG_VERSION}")
    FetchContent_Declare(spdlog
            GIT_REPOSITORY https://github.com/gabime/spdlog.git
            GIT_TAG v${SPDLOG_VERSION}
    )
    set(SPDLOG_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(SPDLOG_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(spdlog)
endif()
