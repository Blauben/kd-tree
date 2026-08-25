# Adapted from https://devblogs.microsoft.com/cppblog/clear-functional-c-documentation-with-sphinx-breathe-doxygen-cmake/
# Include Doxygen
find_package(Doxygen REQUIRED)
# Include Sphinx
include(sphinx)

# Collect the source/ header files
file(GLOB_RECURSE KD_TREE_DOC
        "${KD_TREE_SOURCE_DIR}/src/*.h"
        "${KD_TREE_SOURCE_DIR}/src/*.cpp"
)

set(DOXYGEN_INPUT_DIR ${KD_TREE_SOURCE_DIR}/src)
set(DOXYGEN_OUTPUT_DIR ${KD_TREE_BINARY_DIR}/doxygen)
set(DOXYGEN_INDEX_FILE ${DOXYGEN_OUTPUT_DIR}/xml/index.xml)
set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/source/Doxyfile.in)
set(DOXYFILE_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

# Set the Doxygen input and output directories in the Doxyfile.in
configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

# Doxygen won't create this for us
file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT_DIR})

# Only regenerate Doxygen when the Doxyfile.in or public headers change
add_custom_command(OUTPUT ${DOXYGEN_INDEX_FILE}
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
        COMMENT "Generating docs"
        VERBATIM)

# Nice named target so we can run the job easily
add_custom_target(Doxygen ALL DEPENDS ${DOXYGEN_INDEX_FILE} ${KD_TREE_DOC})

set(SPHINX_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/source)
set(SPHINX_BUILD ${KD_TREE_BINARY_DIR}/html)
set(SPHINX_INDEX_FILE ${SPHINX_BUILD}/index.html)

# Sphinx target depends on Recurse_rst
add_custom_target(Sphinx ALL
        COMMAND ${SPHINX_EXECUTABLE} -W -b html
        -Dbreathe_projects.KDTree=${DOXYGEN_OUTPUT_DIR}/xml
        ${SPHINX_SOURCE} ${SPHINX_BUILD}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        DEPENDS Doxygen
        COMMENT "Generating documentation with Sphinx"
)

message(STATUS "Installing KDTree package in virtual environment for Sphinx documentation. This may take a moment...")

execute_process(
    COMMAND ${VENV_PYTHON} -m pip install ${KD_TREE_SOURCE_DIR}
            --config-settings=cmake.args=--preset
            --config-settings=cmake.args=python
            -v
    RESULT_VARIABLE INSTALL_RESULT
    OUTPUT_VARIABLE INSTALL_OUTPUT
    ERROR_VARIABLE INSTALL_ERROR
    ECHO_OUTPUT_VARIABLE
)

if(NOT INSTALL_RESULT EQUAL 0)
    message(STATUS "pip install error: ${INSTALL_ERROR}")
    message(WARNING "Failed to install KDTree package in virtual environment. Python documentation may not work correctly.")
endif()
