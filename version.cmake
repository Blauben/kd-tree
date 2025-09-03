# Get the Git information
get_git_commit_hash(KD_TREE_COMMIT_HASH)
is_git_working_tree_clean(KD_TREE_WORKING_TREE)
get_git_version_tag(KD_TREE_VERSION)

# Append "-modified" to the commit hash if the working tree is not clean
if (NOT ${KD_TREE_WORKING_TREE})
    set(KD_TREE_COMMIT_HASH "${KD_TREE_COMMIT_HASH}+modified")
endif ()

# Configure the output header file
configure_file(
        "src/KDTree/Info.h.in"
        "src/KDTree/Info.h"
)