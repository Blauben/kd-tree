# Only run this once per configure, not once per test target, since the scaled meshes are shared
# between all test/benchmark targets and this file is include()'d from multiple directory scopes.
# A GLOBAL PROPERTY (rather than a CACHE variable) is used so the guard is visible across those
# scopes but still resets on every fresh configure.
get_property(KD_TREE_SCALED_MESHES_GENERATED GLOBAL PROPERTY KD_TREE_SCALED_MESHES_GENERATED SET)
if(KD_TREE_SCALED_MESHES_GENERATED)
    return()
endif()
set_property(GLOBAL PROPERTY KD_TREE_SCALED_MESHES_GENERATED TRUE)

# Canonical list of face/node counts for the scaled Eros/sphere benchmark mesh variants.
# This is the source of truth for all scripts using the scaled meshes, and should only be updated here.
set(KD_TREE_SCALED_MESH_AMOUNTS 1000 1732 3000 5196 9000 15588 27000 46765 81000 140296)

##########################################################
# Generating the scaled benchmark meshes via Python
##########################################################
# Creates ./.venv on demand (see cmake/pyvenv.cmake) and installs the "mesh-gen" dependency
# group (open3d, numpy - see pyproject.toml) into it if open3d isn't importable yet.
# Pinned to the Python version in .python-version (also what CI installs before running
# scale_mesh.py) since open3d doesn't reliably ship wheels for the newest Python right away.
file(STRINGS "${KD_TREE_SOURCE_DIR}/.python-version" PYVENV_PYTHON_VERSION LIMIT_COUNT 1)
include(pyvenv)
unset(PYVENV_PYTHON_VERSION)

function(kd_tree_install_mesh_gen_dependencies)
    message(STATUS "Installing mesh-gen dependency group (open3d, numpy) into ${VENV_DIR}...")
    execute_process(
            COMMAND "${VENV_PYTHON}" -m pip install --group "${KD_TREE_SOURCE_DIR}/pyproject.toml:mesh-gen"
            RESULT_VARIABLE PIP_INSTALL_RESULT
            OUTPUT_VARIABLE PIP_INSTALL_OUTPUT
            ERROR_VARIABLE PIP_INSTALL_ERROR
    )
    if(NOT PIP_INSTALL_RESULT EQUAL 0)
        message(STATUS "pip install output: ${PIP_INSTALL_OUTPUT}")
        message(STATUS "pip install error: ${PIP_INSTALL_ERROR}")
        message(FATAL_ERROR "Failed to install the mesh-gen dependency group into ${VENV_DIR}. Skipping generation of scaled benchmark meshes (see script/scale_mesh.py).")
    endif()
endfunction()

set(MESH_SCALING_PYTHON_EXECUTABLE "${VENV_PYTHON}")

execute_process(
        COMMAND "${MESH_SCALING_PYTHON_EXECUTABLE}" -c "import open3d"
        RESULT_VARIABLE MESH_SCALING_OPEN3D_MISSING
        OUTPUT_QUIET ERROR_QUIET
)

if(NOT MESH_SCALING_OPEN3D_MISSING EQUAL 0)
    message(STATUS "open3d not found in ${VENV_DIR}, installing the mesh-gen dependency group...")
    kd_tree_install_mesh_gen_dependencies()
endif()

# Generate scaled meshes logic
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

add_custom_target(scale_test_meshes
        COMMENT "Generating scaled benchmark meshes for tests via scale_mesh.py"
)

# Adds a custom command + target that generates scaled variants of one source mesh via
# scale_mesh.py, and hooks it up as a dependency of scale_test_meshes.
#   mesh_name:   base name of the mesh in resources/, e.g. "Eros" or "CubeXDivided"
#   mesh_file_format: either "node-face" (for tetgen .node/.face pairs) or "ply" (for single .ply files)
function(kd_tree_add_scaled_mesh_target mesh_name mesh_file_format)
    # dependening on the mesh_file_format, generate the appropriate command line arguments for
    # scale_mesh.py, the source files to copy into WORKING_DIRECTORY beforehand, and the list of
    # expected output files (scaled meshes) to be used as the custom command's OUTPUT.
    # scale_mesh.py always names its outputs "<input-basename>_scaled-<amount>.<ext>" (see
    # write_to_file()/o3d.io.write_triangle_mesh() in script/scale_mesh.py) - this must match.
    set(source_files "")
    set(scaled_mesh_output_files "")
    if (${mesh_file_format} STREQUAL "node-face")
        set(script_args --node-face ${mesh_name}.node ${mesh_name}.face)
        list(APPEND source_files "${mesh_name}.node" "${mesh_name}.face")
        foreach(face_amount ${MESH_FACE_AMOUNTS})
            list(APPEND scaled_mesh_output_files "${SCALED_MESH_DIR}/${mesh_name}_scaled-${face_amount}.node")
            list(APPEND scaled_mesh_output_files "${SCALED_MESH_DIR}/${mesh_name}_scaled-${face_amount}.face")
        endforeach()
    elseif(${mesh_file_format} STREQUAL "ply")
        set(script_args --ply ${mesh_name}.ply)
        list(APPEND source_files "${mesh_name}.ply")
        foreach(face_amount ${MESH_FACE_AMOUNTS})
            list(APPEND scaled_mesh_output_files "${SCALED_MESH_DIR}/${mesh_name}_scaled-${face_amount}.ply")
        endforeach()
    else()
        message(FATAL_ERROR "Invalid mesh_file_format ${mesh_file_format} for mesh ${mesh_name}. Must be 'node-face' or 'ply'.")
    endif()
    list(TRANSFORM source_files PREPEND "${MESH_SOURCE_DIR}/" OUTPUT_VARIABLE source_file_paths)

    add_custom_command(
            OUTPUT ${scaled_mesh_output_files}
            # scale_mesh.py resolves its input filenames relative to WORKING_DIRECTORY, so the
            # source mesh has to be copied there first.
            COMMAND ${CMAKE_COMMAND} -E copy ${source_file_paths} "${SCALED_MESH_DIR}"
            COMMAND ${MESH_SCALING_PYTHON_EXECUTABLE} "${MESH_SCALE_SCRIPT}" ${script_args} --face-amounts ${MESH_FACE_AMOUNTS}
            WORKING_DIRECTORY "${SCALED_MESH_DIR}"
            DEPENDS ${source_file_paths} "${MESH_SCALE_SCRIPT}"
            COMMENT "Generating scaled meshes for ${mesh_name} via scale_mesh.py"
            VERBATIM
    )
    add_custom_target(generate_${mesh_name}_scaled_meshes
            DEPENDS ${scaled_mesh_output_files}
    )
    add_dependencies(scale_test_meshes generate_${mesh_name}_scaled_meshes)
endfunction()

# Base names of the source meshes in resources/ to generate scaled versions of, grouped by the
# format scale_mesh.py expects them in: node/face pairs (tetgen format) vs single .ply files.
set(MESH_SCALING_BASE_NAMES_NODE_FACE "Eros" "sphere")
set(MESH_SCALING_BASE_NAMES_PLY "4179toutatis.tab" "67P_ESA_NAVCAM_Jul2015data_256k" "CubeXDivided" "MU69_Merged" "Object_25143_Itokawa_200k" "SHAPE_SFM_3M_v20180804" "TetgenAdapterTestReadSimple" "a8567.tab" "hartley2_2012_cart")

message(STATUS "Generating scaled test mesh targets")
foreach(mesh_name ${MESH_SCALING_BASE_NAMES_NODE_FACE})
    kd_tree_add_scaled_mesh_target("${mesh_name}" "node-face")
endforeach()

foreach(mesh_name ${MESH_SCALING_BASE_NAMES_PLY})
    kd_tree_add_scaled_mesh_target("${mesh_name}" "ply")
endforeach()
