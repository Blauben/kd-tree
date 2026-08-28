.. _cmake:

CMake Reference
===============

This page documents the project-specific CMake configuration from
``CMakeLists.txt`` and ``CMakePresets.json``.

Recommended: workflow-first builds
----------------------------------

Use workflow presets as the default entry point. They run the expected
configure/build/test sequence and reduce manual flag drift.

.. code-block:: bash

   # Release builds
   cmake --workflow --preset build-release-default
   cmake --workflow --preset build-release-tbb
   cmake --workflow --preset build-release-omp

   # Test, Python interface, and docs
   cmake --workflow --preset build-and-test-release
   cmake --workflow --preset build-and-test-debug
   cmake --workflow --preset build-python-interface
   cmake --workflow --preset build-docs

   # Benchmarks (builds and runs KDTree_time)
   cmake --workflow --preset build-and-test-benchmarks

Project Options
---------------

+--------------------------------------+--------------------------------------------------------+-------------------------------------------------------------+
| Option                               | Default                                                | Description                                                 |
+======================================+========================================================+=============================================================+
| ``BUILD_KD_TREE_EXECUTABLE``         | ``ON`` (``OFF`` if fetched via ``FetchContent``)       | Build the ``KDTree`` CLI executable from ``src/main.cpp``.  |
+--------------------------------------+--------------------------------------------------------+-------------------------------------------------------------+
| ``BUILD_KD_TREE_LIBRARY``            | ``ON``                                                 | Build the core C++ library target ``KDTree_lib``.           |
+--------------------------------------+--------------------------------------------------------+-------------------------------------------------------------+
| ``BUILD_KD_TREE_PYTHON_INTERFACE``   | ``OFF``                                                | Build the nanobind module ``scikdtree``.                    |
+--------------------------------------+--------------------------------------------------------+-------------------------------------------------------------+
| ``INSTALL_KD_TREE_PYTHON_INTERFACE`` | ``OFF``                                                | Enable install rules for the Python interface/package.      |
+--------------------------------------+--------------------------------------------------------+-------------------------------------------------------------+
| ``BUILD_KD_TREE_TESTS``              | ``OFF``                                                | Build unit tests (target ``KDTree_test``).                  |
+--------------------------------------+--------------------------------------------------------+-------------------------------------------------------------+
| ``KD_TREE_FORCE_GTEST_FETCH``        | ``OFF``                                                | Force fetching GoogleTest even if a local install exists.   |
+--------------------------------------+--------------------------------------------------------+-------------------------------------------------------------+
| ``BUILD_KD_TREE_DOCS``               | ``OFF``                                                | Build Sphinx/Doxygen documentation.                         |
+--------------------------------------+--------------------------------------------------------+-------------------------------------------------------------+
| ``BUILD_KD_TREE_TIME_MEASUREMENT``   | ``OFF``                                                | Build benchmark executable ``KDTree_time``.                 |
+--------------------------------------+--------------------------------------------------------+-------------------------------------------------------------+
| ``ENABLE_IWYU``                      | ``OFF``                                                | Enable include-what-you-use integration during compilation. |
+--------------------------------------+--------------------------------------------------------+-------------------------------------------------------------+
|| ``KD_TREE_PARALLELIZATION``         || ``CPP``                                               || Backend for Thrust host/device path: ``CPP``, ``OMP``,     |
||                                     ||                                                       || ``TBB``.                                                   |
+--------------------------------------+--------------------------------------------------------+-------------------------------------------------------------+
|| ``KD_TREE_LOGGING_LEVEL``           || Active logging level: ``TRACE``, ``DEBUG``, ``INFO``, ||                                                            |
||                                     ||                                                       || ``WARN``, ``ERROR``, ``CRITICAL``, ``OFF``.                |
+--------------------------------------+--------------------------------------------------------+-------------------------------------------------------------+

``BUILD_KD_TREE_LIBRARY`` is a dependent option and is forced ``ON`` when
the executable or tests are enabled.

``BUILD_KD_TREE_TIME_MEASUREMENT`` is ``OFF`` in ``test-release``/``test-debug``,
so the default test workflows don't compile the benchmark executable on every
push. It's built and exercised separately via ``release-debug-symbols`` /
``build-and-test-benchmarks``.

Standard CMake Variables
------------------------

+----------------------+---------------------------------------------------------------+
| Variable             | Description                                                   |
+======================+===============================================================+
| ``CMAKE_BUILD_TYPE`` | Used for single-config generators (for example Ninja).        |
|                      | Presets set ``Release`` or ``Debug`` where appropriate.       |
|                      | Multi-config generators ignore this at configure time.        |
+----------------------+---------------------------------------------------------------+

Presets Overview
----------------

Configure presets (from ``CMakePresets.json``):

+----------------+-------------------------------+-----------------------------------------------------------+
| Preset         | Inherits                      | Main purpose                                              |
+================+===============================+===========================================================+
| ``release``    | ``base``                      | Standard release build (CPP backend).                     |
+----------------+-------------------------------+-----------------------------------------------------------+
| ``debug``      | ``base``                      | Debug build, ``KD_TREE_LOGGING_LEVEL=DEBUG``.             |
+----------------+-------------------------------+-----------------------------------------------------------+
| ``release-omp``| ``release``                   | Release build with OpenMP backend.                        |
+----------------+-------------------------------+-----------------------------------------------------------+
| ``release-tbb``| ``release``                   | Release build with TBB backend.                           |
+----------------+-------------------------------+-----------------------------------------------------------+
| ``test-release``| ``release-tbb``              | Test build in Release with ``BUILD_KD_TREE_TESTS=ON``.    |
+----------------+-------------------------------+-----------------------------------------------------------+
| ``test-debug`` | ``debug``                     | Test build in Debug with ``BUILD_KD_TREE_TESTS=ON``.      |
+----------------+-------------------------------+-----------------------------------------------------------+
| ``python``     | ``release``                   | Python interface (CPP backend), CLI off.                  |
+----------------+-------------------------------+-----------------------------------------------------------+
| ``python-tbb`` | ``python``                    | Python interface with TBB backend.                        |
+----------------+-------------------------------+-----------------------------------------------------------+
| ``python-omp`` | ``python``                    | Python interface with OpenMP backend.                     |
+----------------+-------------------------------+-----------------------------------------------------------+
| ``docs``       | ``base``                      | Docs-only configure preset.                               |
+----------------+-------------------------------+-----------------------------------------------------------+
| ``release-debug-symbols``      | ``base``     | ``RelWithDebInfo`` build with tests and the benchmark     |
|                |                               | executable (``BUILD_KD_TREE_TIME_MEASUREMENT=ON``).       |
+----------------+-------------------------------+-----------------------------------------------------------+

Build presets:

- ``release``, ``debug``, ``release-omp``, ``release-tbb``
- ``test-release``, ``test-debug``
- ``python-cpp``, ``python-omp``, ``python-tbb``
- ``docs-build`` (build target: ``Sphinx``)
- ``release-debug-symbols``

Test presets:

- ``test-release``
- ``test-debug``
- ``test-release-debug-symbols``

Workflow presets:

- ``build-release-default``
- ``build-release-omp``
- ``build-release-tbb``
- ``build-and-test-release``
- ``build-and-test-debug``
- ``build-and-test-benchmarks``
- ``build-python-interface``
- ``build-docs``

Typical Commands
----------------

Preferred: run workflows directly:

.. code-block:: bash

   # cmake --workflow --preset <preset-name>
   cmake --workflow --preset build-release-default
   cmake --workflow --preset build-release-tbb
   cmake --workflow --preset build-release-omp
   cmake --workflow --preset build-and-test-release
   cmake --workflow --preset build-and-test-debug
   cmake --workflow --preset build-and-test-benchmarks
   cmake --workflow --preset build-python-interface
   cmake --workflow --preset build-docs

Alternative: manual configure/build (only when custom flags are needed):

.. code-block:: bash

   cmake -S . -B build/custom -DCMAKE_BUILD_TYPE=Release -DKD_TREE_PARALLELIZATION=OMP -DKD_TREE_LOGGING_LEVEL=INFO -DBUILD_KD_TREE_EXECUTABLE=ON -DBUILD_KD_TREE_TESTS=OFF
   cmake --build build/custom
