###############################
# Building the KDTree Libray
################################
if (BUILD_KD_TREE_LIBRARY AND NOT TARGET ${PROJECT_NAME}_lib)
    message(STATUS "Building the KD Tree Library")

    add_subdirectory(KDTree)

        # Adds the include paths for build and install. Use generator expressions
        # so exported/installed targets do not contain absolute source/build dirs.
        target_include_directories(${PROJECT_NAME}_lib PUBLIC
            $<BUILD_INTERFACE:${KD_TREE_SOURCE_DIR}/src>
            $<BUILD_INTERFACE:${KD_TREE_BINARY_DIR}/src>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/KDTree>
        )

    # Link libraries needed in all targets
    target_link_libraries(${PROJECT_NAME}_lib
            PUBLIC
            tetgen::tetgen
            spdlog::$<IF:$<AND:$<PLATFORM_ID:Darwin>,$<CXX_COMPILER_ID:GNU>>,spdlog_header_only,spdlog>
            thrust::thrust_kdtree
    )
endif ()
#####################################
# Building the KDTree Executable
#####################################
if (BUILD_KD_TREE_EXECUTABLE AND NOT TARGET ${PROJECT_NAME})
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
if (BUILD_KD_TREE_PYTHON_INTERFACE AND NOT TARGET scikdtree)
    message(STATUS "Building the KD Tree Python Interface")

    # Include nanobind
    include(nanobind)

    add_subdirectory(KDTreePython)

    target_link_libraries(scikdtree PUBLIC ${PROJECT_NAME}_lib)
endif ()

#####################################################
# Building the KDTree Time Measurement Executable
#####################################################
if (BUILD_KD_TREE_TIME_MEASUREMENT AND NOT TARGET ${PROJECT_NAME}_time)
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
