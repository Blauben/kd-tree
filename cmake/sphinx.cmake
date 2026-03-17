# Adapted from https://devblogs.microsoft.com/cppblog/clear-functional-c-documentation-with-sphinx-breathe-doxygen-cmake/

# Set up a virtual environment variables with Sphinx, Breathe, and sphinx_rtd_theme
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

function(install_sphinx)
    message(STATUS "Installing Sphinx, Breathe, and sphinx_rtd_theme...")
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
endfunction()

function(validate_sphinx_executable)
    if(NOT SPHINX_EXECUTABLE OR NOT EXISTS "${SPHINX_EXECUTABLE}")
        set(SPHINX_EXECUTABLE "SPHINX_EXECUTABLE-NOTFOUND" CACHE FILEPATH "Path to sphinx-build executable" FORCE)
        return()
    endif()

    execute_process(
        COMMAND "${SPHINX_EXECUTABLE}" --version
        RESULT_VARIABLE SPHINX_VERSION_RESULT
        OUTPUT_VARIABLE SPHINX_VERSION_OUTPUT
        ERROR_VARIABLE SPHINX_VERSION_ERROR
    )

    if(NOT SPHINX_VERSION_RESULT EQUAL 0)
        message(STATUS "sphinx-build validation output: ${SPHINX_VERSION_OUTPUT}")
        message(STATUS "sphinx-build validation error: ${SPHINX_VERSION_ERROR}")
        set(SPHINX_EXECUTABLE "SPHINX_EXECUTABLE-NOTFOUND" CACHE FILEPATH "Path to sphinx-build executable" FORCE)
    endif()
endfunction()

function(install_sphinx_virtual)
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
    endif ()

    # Prevent stale CMake cache entries from reporting a removed executable as found.
    unset(SPHINX_EXECUTABLE)
    unset(SPHINX_EXECUTABLE CACHE)

    find_program(SPHINX_EXECUTABLE
            NAMES sphinx-build sphinx-build.exe
            PATHS "${VENV_BIN_DIR}"
            NO_DEFAULT_PATH
            DOC "Path to sphinx-build executable")

    validate_sphinx_executable()

    if(SPHINX_EXECUTABLE)
        message(STATUS "sphinx-build already available in virtual environment at: ${SPHINX_EXECUTABLE}")
    else()
        message(STATUS "sphinx-build not found in virtual environment, installing Sphinx packages...")
        install_sphinx()
        find_program(SPHINX_EXECUTABLE
                NAMES sphinx-build sphinx-build.exe
                PATHS "${VENV_BIN_DIR}"
                NO_DEFAULT_PATH
                DOC "Path to sphinx-build executable")
        validate_sphinx_executable()
        message(STATUS "Sphinx environment setup complete")
    endif()
endfunction()

install_sphinx_virtual()

# Handle standard arguments to find_package like REQUIRED and QUIET
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Sphinx
        REQUIRED_VARS SPHINX_EXECUTABLE
        FAIL_MESSAGE "Failed to find a working sphinx-build executable in ${VENV_BIN_DIR}")
