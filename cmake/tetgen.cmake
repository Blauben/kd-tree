include(FetchContent)

if(NOT TARGET tetgen_lib)
        message(STATUS "Setting up tetgen")
        # taken from: https://github.com/libigl/tetgen

        set(CMAKE_POLICY_VERSION_MINIMUM 3.10)
        set(TETGEN_SRC_DIR ${KD_TREE_BINARY_DIR}/_deps/tetgen-src)
        set(TETGEN_BUILD_DIR ${KD_TREE_BINARY_DIR}/_deps/tetgen-build)

        FetchContent_Declare(tetgen
                URL ${KD_TREE_SOURCE_DIR}/lib/tetgen_lib.zip
                URL_HASH SHA256=ab0fb45d9f824b5990e20747b10a24fd0c5c06c794e791fabe6241281f927dc7
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
