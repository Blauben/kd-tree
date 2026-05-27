# Get the Git information
get_git_commit_hash(KD_TREE_COMMIT_HASH)
is_git_working_tree_clean(KD_TREE_WORKING_TREE)

# During Python builds, scikit-build-core exposes the wheel version as SKBUILD_PROJECT_VERSION.
# Fall back to Git tags for standalone CMake builds.
if (DEFINED SKBUILD_PROJECT_VERSION AND NOT SKBUILD_PROJECT_VERSION STREQUAL "")
    set(KD_TREE_VERSION "${SKBUILD_PROJECT_VERSION}")
else ()
    get_git_version_tag(KD_TREE_VERSION)
endif ()

# Append "-modified" to the commit hash if the working tree is not clean
if (NOT ${KD_TREE_WORKING_TREE})
    set(KD_TREE_COMMIT_HASH "${KD_TREE_COMMIT_HASH}+modified")
endif ()

# Configure the output header file
file(MAKE_DIRECTORY "${KD_TREE_BINARY_DIR}/src/KDTree")
configure_file(
        "${KD_TREE_SOURCE_DIR}/src/KDTree/Info.h.in"
        "${KD_TREE_BINARY_DIR}/src/KDTree/Info.h"
)

set(KD_TREE_GENERATED_INFO_HEADER "${KD_TREE_BINARY_DIR}/src/KDTree/Info.h")
