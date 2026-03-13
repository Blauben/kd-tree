.. _KDTree:

C++ Documentation
========================

KDTree
^^^^^^

.. doxygenclass:: kdtree::KDTree
   :project: KDTree
   :members:
   :undoc-members:

Tree Nodes
^^^^^^^^^^

.. doxygenclass:: kdtree::TreeNode
   :project: KDTree
   :members:
   :undoc-members:

.. doxygenclass:: kdtree::SplitNode
   :project: KDTree
   :members:
   :undoc-members:

.. doxygenclass:: kdtree::LeafNode
   :project: KDTree
   :members:
   :undoc-members:

Plane Selection Algorithms
---------------------------

Base Algorithm
^^^^^^^^^^^^^^

.. doxygenclass:: kdtree::PlaneSelectionAlgorithm
   :project: KDTree
   :members:
   :undoc-members:

Implementations
^^^^^^^^^^^^^^^

.. doxygenclass:: kdtree::LogNPlane
   :project: KDTree
   :members:
   :undoc-members:

.. doxygenclass:: kdtree::LogNSquaredPlane
   :project: KDTree
   :members:
   :undoc-members:

.. doxygenclass:: kdtree::SquaredPlane
   :project: KDTree
   :members:
   :undoc-members:

.. doxygenclass:: kdtree::NoTreePlane
   :project: KDTree
   :members:
   :undoc-members:

.. doxygenclass:: kdtree::PlaneEventAlgorithm
   :project: KDTree
   :members:
   :undoc-members:

Factory
^^^^^^^

.. doxygenclass:: kdtree::PlaneSelectionAlgorithmFactory
   :project: KDTree
   :members:
   :undoc-members:

Geometry and Data Structures
-----------------------------
.. doxygenclass:: kdtree::GeometryObject
   :project: KDTree
   :members:
   :undoc-members:

.. doxygenstruct:: kdtree::Box
   :project: KDTree
   :members:
   :undoc-members:

.. doxygenstruct:: kdtree::Plane
   :project: KDTree
   :members:
   :undoc-members:

.. doxygenstruct:: kdtree::PlaneEvent
   :project: KDTree
   :members:
   :undoc-members:

.. doxygenstruct:: kdtree::SplitParam
   :project: KDTree
   :members:
   :undoc-members:

.. doxygenstruct:: kdtree::Meshes
   :project: KDTree
   :members:
   :undoc-members:

Utilities
---------
.. doxygenclass:: kdtree::ShapeCounter
   :project: KDTree
   :members:
   :undoc-members:

.. doxygenclass:: kdtree::TetgenAdapter
   :project: KDTree
   :members:
   :undoc-members:

Utility Namespaces
^^^^^^^^^^^^^^^^^^

.. doxygennamespace:: kdtree::util
   :project: KDTree
   :members:
   :undoc-members:

.. doxygennamespace:: kdtree::TreeNodeFactory
   :project: KDTree
   :members:
   :undoc-members:


Indices and tables
------------------
* :ref:`genindex`
* :ref:`search`
