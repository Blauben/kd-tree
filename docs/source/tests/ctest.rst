CTest (C++ Tests)
=================

This project uses GoogleTest with CTest integration. The test target is
``KDTree_test`` and tests are registered via ``gtest_discover_tests``.

Preset-Based Workflow (Recommended)
-----------------------------------

.. code-block:: bash

   cmake --workflow --preset build-and-test

Manual Workflow
---------------

.. code-block:: bash

   cmake -S . -B build/test -DCMAKE_BUILD_TYPE=Debug -DBUILD_KD_TREE_TESTS=ON
   cmake --build build/test
   ctest --test-dir build/test --output-on-failure

Run a Specific CTest Case
-------------------------

.. code-block:: bash

   ctest --test-dir build/test -R KDTree --output-on-failure
