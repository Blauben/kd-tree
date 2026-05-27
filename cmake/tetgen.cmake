include(FetchContent)

if(NOT TARGET tetgen_lib) 
        message(STATUS "Setting up tetgen")
        # taken from: https://github.com/libigl/tetgen

        set(CMAKE_POLICY_VERSION_MINIMUM 3.10)
        set(TETGEN_SRC_DIR ${KD_TREE_BINARY_DIR}/_deps/tetgen-src)
        set(TETGEN_BUILD_DIR ${KD_TREE_BINARY_DIR}/_deps/tetgen-build)

        file(ARCHIVE_EXTRACT
                INPUT ${KD_TREE_SOURCE_DIR}/lib/tetgen_lib.zip
                DESTINATION ${TETGEN_SRC_DIR}
        )

        add_subdirectory(${TETGEN_SRC_DIR} ${TETGEN_BUILD_DIR} EXCLUDE_FROM_ALL)

        mark_as_advanced(FORCE BUILD_EXECUTABLE BUILD_LIBRARY)

        # Add the modified version of the tetgen library
        add_library(tetgen_lib STATIC
                ${tetgen_SOURCE_DIR}/tetgen.cxx
                ${tetgen_SOURCE_DIR}/predicates.cxx
        )

        # Define the TETLIBRARY macro for usage
        target_compile_definitions(tetgen_lib PRIVATE -DTETLIBRARY)

                # Include the tetgen source directory for the library. Use BUILD/INSTALL
                # interface generator expressions to avoid exporting absolute paths.
                target_include_directories(tetgen_lib INTERFACE
                        $<BUILD_INTERFACE:${tetgen_SOURCE_DIR}>
                        $<INSTALL_INTERFACE:include/tetgen>
                )

        # Disable warnings from the library target
        target_compile_options(tetgen_lib PRIVATE -w)

else()
        message(STATUS "tetgen library already exists in the project. Using existing target. CAUTION: library modifications may not be applied.")
endif()

if(NOT TARGET tetgen::tetgen)
        add_library(tetgen::tetgen ALIAS tetgen_lib)
else()
        message(STATUS "tetgen::tetgen target already exists. Using existing target.")
endif()