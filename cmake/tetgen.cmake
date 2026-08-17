include(FetchContent)

if(NOT TARGET tetgen_lib)
        message(STATUS "Setting up tetgen")
        # taken from: https://github.com/libigl/tetgen

        set(CMAKE_POLICY_VERSION_MINIMUM 3.10)
        set(TETGEN_SRC_DIR ${KD_TREE_BINARY_DIR}/_deps/tetgen-src)
        set(TETGEN_BUILD_DIR ${KD_TREE_BINARY_DIR}/_deps/tetgen-build)

        FetchContent_Declare(tetgen
                URL ${KD_TREE_SOURCE_DIR}/lib/tetgen_lib.zip
                URL_HASH SHA256=795c17c869b6e7cccabf31ac775304a7837c828a14d06173b25c5276ea65fbf8
                DOWNLOAD_EXTRACT_TIMESTAMP TRUE
       )

       FetchContent_MakeAvailable(tetgen)

        # Include the tetgen source directory for the library. Use BUILD/INSTALL
        # interface generator expressions to avoid exporting absolute paths.
        target_include_directories(tetgen INTERFACE
                $<BUILD_INTERFACE:${tetgen_SOURCE_DIR}>
                $<INSTALL_INTERFACE:include/tetgen>
        )
else()
        message(STATUS "tetgen library already exists in the project. Using existing target. CAUTION: library modifications may not be applied.")
endif()

if(NOT TARGET tetgen::tetgen)
        add_library(tetgen::tetgen ALIAS tetgen)
else()
        message(STATUS "tetgen::tetgen target already exists. Using existing target.")
endif()