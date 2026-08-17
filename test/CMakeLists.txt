# Includes the gtest.cmake
include(gtest)

# includes indicators.cmake for progress bars in tests
include(indicators)

# Recursively collects all the test files in the test Directory
file(GLOB_RECURSE TEST_SRC
        "${KD_TREE_SOURCE_DIR}/test/*.cpp"
        "${KD_TREE_SOURCE_DIR}/tests/*.h"
)

# Creates the Test Target
add_executable(${PROJECT_NAME}_test ${TEST_SRC})

set_target_properties(${PROJECT_NAME}_test PROPERTIES
            ARCHIVE_OUTPUT_DIRECTORY ${KD_TREE_BINARY_DIR}
            LIBRARY_OUTPUT_DIRECTORY ${KD_TREE_BINARY_DIR}
            RUNTIME_OUTPUT_DIRECTORY ${KD_TREE_BINARY_DIR}
)

# Links target breakupModelTest against gtest_main (entry point, so no own main method is required)
target_link_libraries(${PROJECT_NAME}_test
        indicators
        GTest::gtest_main
        GTest::gtest
        GTest::gmock
        ${PROJECT_NAME}_lib
)
kd_tree_copy_runtime_dlls(${PROJECT_NAME}_test)

target_include_directories(${PROJECT_NAME}_test PUBLIC ${KD_TREE_SOURCE_DIR}/src)

# Copies the test resources into a directory next to the test executable
file(COPY "${KD_TREE_SOURCE_DIR}/resources" DESTINATION "${KD_TREE_BINARY_DIR}")

##########################################################
# Generating the scaled benchmark meshes via Python
##########################################################
# Prefers the project's local virtualenv (./.venv), since that is where open3d is expected
# to be installed; falls back to whatever Python3 CMake finds on the system otherwise.
find_program(MESH_SCALING_PYTHON_EXECUTABLE
        NAMES python3 python
        HINTS "${KD_TREE_SOURCE_DIR}/.venv/bin" "${KD_TREE_SOURCE_DIR}/.venv/Scripts"
        NO_DEFAULT_PATH
)
if(NOT MESH_SCALING_PYTHON_EXECUTABLE)
    find_package(Python3 COMPONENTS Interpreter QUIET)
    set(MESH_SCALING_PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
endif()

if(MESH_SCALING_PYTHON_EXECUTABLE)
    execute_process(
            COMMAND ${MESH_SCALING_PYTHON_EXECUTABLE} -c "import open3d"
            RESULT_VARIABLE MESH_SCALING_OPEN3D_MISSING
            OUTPUT_QUIET ERROR_QUIET
    )
endif()

if(MESH_SCALING_PYTHON_EXECUTABLE AND MESH_SCALING_OPEN3D_MISSING EQUAL 0)
    message(STATUS "Python3 with open3d found at ${MESH_SCALING_PYTHON_EXECUTABLE}. Scaled benchmark meshes will be generated via scale_mesh.py")

    set(MESH_SOURCE_DIR "${KD_TREE_SOURCE_DIR}/resources")
    set(SCALED_MESH_DIR "${CMAKE_CURRENT_BINARY_DIR}/resources")
    set(MESH_SCALE_SCRIPT "${KD_TREE_SOURCE_DIR}/script/scale_mesh.py")

    # Matches the face amounts hardcoded in script/scale_mesh.py: round(1000 * sqrt(3)^k) for k in range(10)
    set(MESH_FACE_AMOUNTS 1000 1732 3000 5196 9000 15588 27000 46765 81000 140296)

    # Base names of the source meshes in script/mesh (as <name>.node / <name>.face) to generate scaled versions of
    set(MESH_SCALING_BASE_NAMES Eros sphere)
    foreach(mesh_name ${MESH_SCALING_BASE_NAMES})
        set(SCALED_MESH_FILES "")
        foreach(amount ${MESH_FACE_AMOUNTS})
            list(APPEND SCALED_MESH_FILES
                    "${SCALED_MESH_DIR}/${mesh_name}_scaled-${amount}.node"
                    "${SCALED_MESH_DIR}/${mesh_name}_scaled-${amount}.face"
            )
        endforeach()

        add_custom_command(
                OUTPUT ${SCALED_MESH_FILES}
                COMMAND ${CMAKE_COMMAND} -E make_directory "${SCALED_MESH_DIR}"
                COMMAND ${CMAKE_COMMAND} -E copy "${MESH_SOURCE_DIR}/${mesh_name}.node" "${MESH_SOURCE_DIR}/${mesh_name}.face" "${SCALED_MESH_DIR}"
                COMMAND ${MESH_SCALING_PYTHON_EXECUTABLE} "${MESH_SCALE_SCRIPT}" "${mesh_name}.node" "${mesh_name}.face"
                WORKING_DIRECTORY "${SCALED_MESH_DIR}"
                DEPENDS "${MESH_SOURCE_DIR}/${mesh_name}.node" "${MESH_SOURCE_DIR}/${mesh_name}.face" "${MESH_SCALE_SCRIPT}"
                COMMENT "Generating scaled ${mesh_name} meshes for benchmarking via scale_mesh.py"
                VERBATIM
        )

        add_custom_target(generate_${mesh_name}_scaled_meshes DEPENDS ${SCALED_MESH_FILES})
        add_dependencies(${PROJECT_NAME}_test generate_${mesh_name}_scaled_meshes)
    endforeach()
else()
    message(STATUS "Python3 with open3d not found (checked ${KD_TREE_SOURCE_DIR}/.venv and the system Python3). Skipping generation of scaled benchmark meshes (see script/scale_mesh.py).")
endif()

include(GoogleTest)

# Adds the Tests to CTest by querying the test target executable
gtest_discover_tests(${PROJECT_NAME}_test
        WORKING_DIRECTORY ${KD_TREE_BINARY_DIR}
        DISCOVERY_TIMEOUT 120
)
