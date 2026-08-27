# Only run this once per build, not once per test target, since the scaled meshes are shared between all test/benchmark targets.
if(KD_TREE_SCALED_MESHES_GENERATED)
    return()
else()
    set(KD_TREE_SCALED_MESHES_GENERATED TRUE CACHE INTERNAL "Whether the scaled benchmark meshes have been generated yet")
endif()

# Canonical list of face/node counts for the scaled Eros/sphere benchmark mesh variants.
# This is the source of truth for all scripts using the scaled meshes, and should only be updated here.
set(KD_TREE_SCALED_MESH_AMOUNTS 1000 1732 3000 5196 9000 15588 27000 46765 81000 140296)

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
    set(SCALED_MESH_DIR "${KD_TREE_BINARY_DIR}/resources")
    set(MESH_SCALE_SCRIPT "${KD_TREE_SOURCE_DIR}/script/scale_mesh.py")

    # Created eagerly at configure time so it already exists once the custom commands below
    # run; the Makefile generator cd's into WORKING_DIRECTORY before executing any of a custom
    # command's steps, so creating it as one of those steps is too late.
    file(MAKE_DIRECTORY "${SCALED_MESH_DIR}")

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
