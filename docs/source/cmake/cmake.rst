.. _cmake::

CMake Reference
===============

This page documents the project-specific CMake configuration from
``CMakeLists.txt`` and ``CMakePresets.json``.

Project Options
---------------

+-------------------------------------+---------+--------------------------------------------------------------+
| Option                              | Default | Description                                                  |
+=====================================+=========+==============================================================+
| ``BUILD_KD_TREE_EXECUTABLE``        | ``ON``  | Build the ``KDTree`` CLI executable from ``src/main.cpp``.   |
+-------------------------------------+---------+--------------------------------------------------------------+
| ``BUILD_KD_TREE_LIBRARY``           | ``ON``* | Build the core C++ library target ``KDTree_lib``.            |
+-------------------------------------+---------+--------------------------------------------------------------+
| ``BUILD_KD_TREE_PYTHON_INTERFACE``  | ``OFF`` | Build nanobind module ``scikdtree``.                         |
+-------------------------------------+---------+--------------------------------------------------------------+
| ``INSTALL_KD_TREE_PYTHON_INTERFACE``| ``OFF`` | Enable install rules for the Python package/module.          |
+-------------------------------------+---------+--------------------------------------------------------------+
| ``BUILD_KD_TREE_TESTS``             | ``OFF`` | Build unit-test executable ``KDTree_test``.                  |
+-------------------------------------+---------+--------------------------------------------------------------+
| ``BUILD_KD_TREE_DOCS``              | ``OFF`` | Build Sphinx/Doxygen documentation.                          |
+-------------------------------------+---------+--------------------------------------------------------------+
| ``BUILD_KD_TREE_TIME_MEASUREMENT``  | ``OFF`` | Build benchmark executable ``KDTree_time``.                  |
+-------------------------------------+---------+--------------------------------------------------------------+
| ``ENABLE_IWYU``                     | ``OFF`` | Enable include-what-you-use checks during compilation.       |
|                                     |         | Requires include-what-you use to be installed on the system. |
+-------------------------------------+---------+--------------------------------------------------------------+
| ``KD_TREE_PARALLELIZATION``         | ``CPP`` | Backend for Thrust host/device path: ``CPP``, ``OMP``,       |
|                                     |         | ``TBB``.                                                     |
+-------------------------------------+---------+--------------------------------------------------------------+
| ``KD_TREE_LOGGING_LEVEL``           | ``INFO``| Active logging level: ``TRACE``, ``DEBUG``, ``INFO``,        |
|                                     |         | ``WARN``, ``ERROR``, ``CRITICAL``, ``OFF``.                  |
+-------------------------------------+---------+--------------------------------------------------------------+

\* ``BUILD_KD_TREE_LIBRARY`` is a dependent option and is forced ``ON`` when
the executable or tests are enabled.

Standard CMake Variables
------------------------

+----------------------+---------------------------------------------------------------+
| Variable             | Description                                                   |
+======================+===============================================================+
| ``CMAKE_BUILD_TYPE`` | Used for single-config generators (for example Ninja).        |
|                      | Presets set ``Release`` or ``Debug`` where appropriate.       |
+----------------------+---------------------------------------------------------------+

Presets Overview
----------------

Configure presets (from ``CMakePresets.json``):

+----------------+-------------------------------+-------------------------------------------+
| Preset         | Inherits                      | Main purpose                              |
+================+===============================+===========================================+
| ``release``    | ``base``                      | Standard release build (CPP backend).     |
+----------------+-------------------------------+-------------------------------------------+
| ``debug``      | ``base``                      | Debug build + ``KD_TREE_LOGGING_LEVEL``   |
|                |                               | set to ``DEBUG``.                         |
+----------------+-------------------------------+-------------------------------------------+
| ``release-omp``| ``release``                   | Release build with OpenMP backend.        |
+----------------+-------------------------------+-------------------------------------------+
| ``release-tbb``| ``release``                   | Release build with TBB backend.           |
+----------------+-------------------------------+-------------------------------------------+
| ``test``       | ``debug``                     | Enables ``BUILD_KD_TREE_TESTS=ON``.       |
+----------------+-------------------------------+-------------------------------------------+
| ``python``     | ``release``                   | Build Python interface, disables CLI exe. |
+----------------+-------------------------------+-------------------------------------------+
| ``python-tbb`` | ``python``                    | Python interface with TBB backend.        |
+----------------+-------------------------------+-------------------------------------------+
| ``python-omp`` | ``python``                    | Python interface with OpenMP backend.     |
+----------------+-------------------------------+-------------------------------------------+

Common build presets:

- ``release``, ``debug``, ``release-omp``, ``release-tbb``
- ``test``
- ``python-cpp``, ``python-omp``, ``python-tbb``

Workflow presets:

- ``build-release-default``
- ``build-release-omp``
- ``build-release-tbb``
- ``build-and-test``
- ``build-python-interface``

Typical Commands
----------------

Build releases using cmake workflow presets:

.. code-block:: bash

   # cmake --workflow --preset <preset-name>
   cmake --workflow --preset build-release-default

Recommended (preset-based) builds:

.. code-block:: bash

   # Standard release (CPP backend)
   cmake --preset release
   cmake --build --preset release

   # Debug with verbose logging
   cmake --preset debug
   cmake --build --preset debug

   # Release with TBB backend
   cmake --preset release-tbb
   cmake --build --preset release-tbb

   # Build and run tests
   cmake --preset test
   cmake --build --preset test
   ctest --preset test

Manual configuration example without presets:

.. code-block:: bash

   cmake -S . -B build/custom \
      -DCMAKE_BUILD_TYPE=Release \
     -DKD_TREE_PARALLELIZATION=OMP \
     -DKD_TREE_LOGGING_LEVEL=INFO \
     -DBUILD_KD_TREE_EXECUTABLE=ON \
     -DBUILD_KD_TREE_TESTS=OFF

   cmake --build build/custom
