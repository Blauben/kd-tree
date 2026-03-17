###############################
# Building the KDTree Libray
################################
if (BUILD_KD_TREE_LIBRARY)
    message(STATUS "Building the KD Tree Library")

    add_subdirectory(KDTree)

    # Adds the include Path PROJECT_SOURCE_DIR/src to the target KDTree
    target_include_directories(${PROJECT_NAME}_lib PUBLIC
            "${CMAKE_CURRENT_SOURCE_DIR}"
            "${CMAKE_CURRENT_BINARY_DIR}"
    )

    # Link libraries needed in all targets
    target_link_libraries(${PROJECT_NAME}_lib
            PUBLIC
            tetgen_lib
            spdlog
            xsimd
            Thrust
    )
endif ()
#####################################
# Building the KDTree Executable
#####################################
if (BUILD_KD_TREE_EXECUTABLE)
    message(STATUS "Building the KD Tree Executable")

    include(cli11)

    # Building the standalone Executable
    add_executable(${PROJECT_NAME} main.cpp)
    set_target_properties(${PROJECT_NAME} PROPERTIES WIN32_EXECUTABLE FALSE)

    # Link executable with library
    target_link_libraries(${PROJECT_NAME} PUBLIC ${PROJECT_NAME}_lib CLI11::CLI11)
    kd_tree_copy_runtime_dlls(${PROJECT_NAME})

    # Place the executable in the top level directory
    set_target_properties(${PROJECT_NAME} PROPERTIES
            ARCHIVE_OUTPUT_DIRECTORY ${KD_TREE_BINARY_DIR}
            LIBRARY_OUTPUT_DIRECTORY ${KD_TREE_BINARY_DIR}
            RUNTIME_OUTPUT_DIRECTORY ${KD_TREE_BINARY_DIR}
    )
endif ()

##########################################
# Building the KDTree Python Interface
##########################################
if (BUILD_KD_TREE_PYTHON_INTERFACE)
    message(STATUS "Building the KD Tree Python Interface")

    # Include nanobind
    include(nanobind)

    add_subdirectory(KDTreePython)

    target_link_libraries(scikdtree PUBLIC ${PROJECT_NAME}_lib)
endif ()

#####################################################
# Building the KDTree Time Measurement Executable
#####################################################
if (BUILD_KD_TREE_TIME_MEASUREMENT)
    message(STATUS "Building the KD Tree Benchmark Executable")

    include(benchmark)

    # Building the standalone Executable
    add_executable(${PROJECT_NAME}_time kd_time_main.cpp)

    # Link executable with library
    target_link_libraries(${PROJECT_NAME}_time PUBLIC ${PROJECT_NAME}_lib benchmark::benchmark)
    kd_tree_copy_runtime_dlls(${PROJECT_NAME}_time)

    # Place the executable in the top level directory
    set_target_properties(${PROJECT_NAME}_time PROPERTIES
            ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
            LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
            RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
    )

    file(COPY ${KD_TREE_SOURCE_DIR}/resources DESTINATION ${PROJECT_BINARY_DIR})

endif ()