Pytest (Python Tests)
=====================

Python tests live under ``test/python``. Pytest discovery is configured in
``pyproject.toml`` (``[tool.pytest.ini_options]``).

Install Test Dependencies
-------------------------

.. code-block:: bash

   # optional: create and activate a virtual environment
   pip install pytest

Build and Install the Python Module
-----------------------------------

Build and install the scikdtree package first:

.. code-block:: bash

   # From repository root
   pip install .

Run Pytest
----------

.. code-block:: bash

   pytest -q

Useful Variants
---------------

.. code-block:: bash

   # Verbose output
   pytest -v

   # Stop on first failure
   pytest -x

   # Parallel execution (requires pytest-xdist)
   pytest -n auto

