###############################
# Building the KD-Tree Library
################################
if (BUILD_KDTREE_LIBRARY)
    message(STATUS "Building the KD-Tree Library")

    # YAML is only required for the library target, but not for e.g. the Python interface
    include(yaml)

    add_subdirectory(KDTree)

    # Adds the include Path PROJECT_SOURCE_DIR/src to the target polyhedralGravity
    target_include_directories(${PROJECT_NAME}_lib PUBLIC
            "${CMAKE_CURRENT_SOURCE_DIR}"
    )

    # Link libraries needed in all targets
    target_link_libraries(${PROJECT_NAME}_lib
            spdlog::spdlog
            yaml-cpp::yaml-cpp
            tetgen_lib
            xsimd
            Thrust
    )
endif ()
#####################################
# Building the KD-Tree Executable
#####################################
if (BUILD_KDTREE_EXECUTABLE)
    message(STATUS "Building the KD-Tree Executable")

    # Building the standalone Executable
    add_executable(${PROJECT_NAME} main.cpp)

    # Link executable with library
    target_link_libraries(${PROJECT_NAME} PUBLIC ${PROJECT_NAME}_lib)

    # Place the executable in the top level directory
    set_target_properties(${PROJECT_NAME} PROPERTIES
            ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
            LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
            RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
    )
endif ()