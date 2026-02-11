# Adapted from https://devblogs.microsoft.com/cppblog/clear-functional-c-documentation-with-sphinx-breathe-doxygen-cmake/

# Set up a virtual environment with Sphinx, Breathe, and sphinx_rtd_theme
set(VENV_DIR "${CMAKE_SOURCE_DIR}/.venv")

# Cross-platform virtual environment paths
if(WIN32)
    set(VENV_BIN_DIR "${VENV_DIR}/Scripts")
    set(VENV_PYTHON "${VENV_BIN_DIR}/python.exe")
    set(VENV_SPHINX "${VENV_BIN_DIR}/sphinx-build.exe")
else()
    set(VENV_BIN_DIR "${VENV_DIR}/bin")
    set(VENV_PYTHON "${VENV_BIN_DIR}/python")
    set(VENV_SPHINX "${VENV_BIN_DIR}/sphinx-build")
endif()

# Check if virtual environment exists, if not create it
if(NOT EXISTS "${VENV_DIR}")
    message(STATUS "Creating Python virtual environment at ${VENV_DIR}")
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    execute_process(
        COMMAND "${Python3_EXECUTABLE}" -m venv "${VENV_DIR}"
        RESULT_VARIABLE VENV_CREATE_RESULT
    )
    if(NOT VENV_CREATE_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to create virtual environment")
    endif()

    # Install required packages
    message(STATUS "Installing Sphinx, Breathe, and sphinx_rtd_theme...")
    execute_process(
        COMMAND "${VENV_PYTHON}" -m pip install --upgrade pip
        RESULT_VARIABLE PIP_UPGRADE_RESULT
        OUTPUT_VARIABLE PIP_UPGRADE_OUTPUT
        ERROR_VARIABLE PIP_UPGRADE_ERROR
    )
    if(NOT PIP_UPGRADE_RESULT EQUAL 0)
        message(STATUS "pip upgrade output: ${PIP_UPGRADE_OUTPUT}")
        message(STATUS "pip upgrade error: ${PIP_UPGRADE_ERROR}")
        message(WARNING "Failed to upgrade pip, continuing anyway...")
    endif()

    execute_process(
        COMMAND "${VENV_PYTHON}" -m pip install sphinx breathe sphinx_rtd_theme
        RESULT_VARIABLE PIP_INSTALL_RESULT
        OUTPUT_VARIABLE PIP_INSTALL_OUTPUT
        ERROR_VARIABLE PIP_INSTALL_ERROR
    )
    if(NOT PIP_INSTALL_RESULT EQUAL 0)
        message(STATUS "pip install output: ${PIP_INSTALL_OUTPUT}")
        message(STATUS "pip install error: ${PIP_INSTALL_ERROR}")
        message(FATAL_ERROR "Failed to install Sphinx packages")
    endif()
    message(STATUS "Sphinx environment setup complete")
endif()

# Look for sphinx-build in virtual environment first, then system-wide
find_program(SPHINX_EXECUTABLE
        NAMES sphinx-build sphinx-build.exe
        PATHS "${VENV_BIN_DIR}"
        NO_DEFAULT_PATH
        DOC "Path to sphinx-build executable")

# If not found in venv, search system-wide
if(NOT SPHINX_EXECUTABLE)
    find_program(SPHINX_EXECUTABLE
            NAMES sphinx-build
            DOC "Path to sphinx-build executable")
endif()

if (NOT SPHINX_EXECUTABLE)
    message(STATUS "sphinx-build not found.")
else ()
    message(STATUS "Found sphinx-build at: ${SPHINX_EXECUTABLE}")
endif ()

include(FindPackageHandleStandardArgs)

# Handle standard arguments to find_package like REQUIRED and QUIET
find_package_handle_standard_args(Sphinx
        "Failed to find sphinx-build executable"
        SPHINX_EXECUTABLE)