# Modify the version with a new release
set(PROJECT_VERSION 3.2.1)
set(KDTREE_VERSION ${PROJECT_VERSION})

# Get the Git information
get_git_commit_hash(KDTREE_COMMIT_HASH)
is_git_working_tree_clean(KDTREE_WORKING_TREE)

# Append "-modified" to the commit hash if the working tree is not clean
if (NOT ${KDTREE_WORKING_TREE})
    set(KDTREE_COMMIT_HASH "${KDTREE_COMMIT_HASH}+modified")
endif ()

# Configure the output header file
configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/src/KDTree/Info.h.in"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/KDTree/Info.h"
)