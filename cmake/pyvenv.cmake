set(VENV_DIR "${KD_TREE_SOURCE_DIR}/.venv")

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

if(NOT EXISTS "${VENV_DIR}")
    message(STATUS "Creating Python virtual environment at ${VENV_DIR}")
    # Set PYVENV_PYTHON_VERSION (e.g. "3.11") before include(pyvenv) to pin the interpreter used
    # to create the venv, e.g. to match a dependency that doesn't yet ship wheels for the newest
    # Python. Left unset, whatever Python3 CMake finds by default is used.
    if(PYVENV_PYTHON_VERSION)
        find_package(Python3 ${PYVENV_PYTHON_VERSION} EXACT COMPONENTS Interpreter REQUIRED)
    else()
        find_package(Python3 COMPONENTS Interpreter REQUIRED)
    endif()
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
