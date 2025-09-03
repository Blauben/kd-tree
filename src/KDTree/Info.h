#pragma once

#include <string_view>

// ##########################################################################################
// Please note! The content of this file is automatically modified by the CMake build system
// which replaces the variables
// ##########################################################################################

namespace kdtree {

    /**
     * The API's version of the polyhedral gravity model's interface.
     * The value is set by the CMake configuration.
     */
    constexpr std::string_view KD_TREE_VERSION = "3.2.1";

    /**
     * The API's parallelization backend of the polyhedral gravity model's interface.
     * The value is set by the CMake configuration.
     */
    constexpr std::string_view KD_TREE_PARALLELIZATION = "CPP";

    /**
     * The commit hash of the currently compiled version.
     * This is appened with '+modified' in case the working tree is not clean.
     * The value is set by the CMake configuration.
     */
    constexpr std::string_view KD_TREE_COMMIT_HASH = "56061977+modified";

    /**
     * The API's Logging Level. Determines the amount of output.
     * The value is set by the CMake configuration.
     */
    constexpr std::string_view KD_TREE_LOGGING_LEVEL = "INFO";

}// namespace kdtree
