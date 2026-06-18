# Adapted from https://devblogs.microsoft.com/cppblog/clear-functional-c-documentation-with-sphinx-breathe-doxygen-cmake/

# Set up a virtual environment variables with Sphinx, Breathe, and sphinx_rtd_theme
include(pyvenv)

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
