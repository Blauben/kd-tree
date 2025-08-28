###############################
# Building the KDTree Libray
################################
if (BUILD_KD_TREE_LIBRARY)
    message(STATUS "Building the KD Tree Library")

    # YAML is only required for the library target, but not for e.g. the Python interface
    # include(yaml) # TODO: review

    add_subdirectory(KDTree)

    # Adds the include Path PROJECT_SOURCE_DIR/src to the target KDTree
    target_include_directories(${PROJECT_NAME}_lib PUBLIC
            "${CMAKE_CURRENT_SOURCE_DIR}"
    )

    # Link libraries needed in all targets
    target_link_libraries(${PROJECT_NAME}_lib
            spdlog::spdlog
            # yaml-cpp::yaml-cpp TODO
            tetgen_lib
            xsimd
            Thrust
    )
endif ()
#####################################
# Building the KDTree Executable
#####################################
if (BUILD_KD_TREE_EXECUTABLE)
    message(STATUS "Building the KD Tree Executable")

    # Building the standalone Executable
    add_executable(${PROJECT_NAME} kd_tree_main.cpp)
    set_target_properties(${PROJECT_NAME} PROPERTIES WIN32_EXECUTABLE FALSE)


    # Link executable with library
    target_link_libraries(${PROJECT_NAME} PUBLIC ${PROJECT_NAME}_lib)

    # Place the executable in the top level directory
    set_target_properties(${PROJECT_NAME} PROPERTIES
            ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
            LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
            RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
    )
endif ()

##########################################
# Building the KDTree Python Interface
##########################################
if (BUILD_KD_TREE_PYTHON_INTERFACE)
    message(STATUS "Building the KD Tree Python Interface")

    # Include pybind11
    include(pybind11)

    add_subdirectory(polyhedralGravityPython) # TODO: build for kd tree

    target_include_directories(KD_TREE PUBLIC
            "${CMAKE_CURRENT_SOURCE_DIR}"
    )

    target_link_libraries(KD_TREE PUBLIC
            spdlog::spdlog
            tetgen_lib
            xsimd
            Thrust
    )
endif ()

#####################################################
# Building the KDTree Time Measurement Executable
#####################################################
if (BUILD_KD_TREE_TIME_MEASUREMENT)
    message(STATUS "Building the KD Tree Executable")

    include(benchmark)

    # Building the standalone Executable
    add_executable(${PROJECT_NAME}_time kd_time_main.cpp)

    # Link executable with library
    target_link_libraries(${PROJECT_NAME}_time PUBLIC ${PROJECT_NAME}_lib benchmark::benchmark)

    # Place the executable in the top level directory
    set_target_properties(${PROJECT_NAME}_time PROPERTIES
            ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
            LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
            RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
    )
endif ()