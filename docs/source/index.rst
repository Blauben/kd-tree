.. KDTree documentation master file, created by
   sphinx-quickstart on Sun Nov 23 23:01:20 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. KDTree documentation master file

Welcome to KDTree's documentation!
==================================

KDTree is a spatial data structure implementation for efficient nearest neighbor searches and spatial queries.

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   overview
   api_reference
   examples

Overview
========

The KDTree library provides efficient spatial indexing and querying capabilities for 3D geometric objects.

Core Classes
============

KDTree
------

.. doxygenclass:: kdtree::KDTree
   :project: KDTree
   :members:
   :undoc-members:

Tree Nodes
----------

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
===========================

Base Algorithm
--------------

.. doxygenclass:: kdtree::PlaneSelectionAlgorithm
   :project: KDTree
   :members:
   :undoc-members:

.. doxygenclass:: kdtree::PlaneSelectionAlgorithm::OptimalPlane
   :project: KDTree
   :members:
   :undoc-members:

Implementations
---------------

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
-------

.. doxygenclass:: kdtree::PlaneSelectionAlgorithmFactory
   :project: KDTree
   :members:
   :undoc-members:

Geometry and Data Structures
=============================

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
=========

.. doxygenclass:: kdtree::ShapeCounter
   :project: KDTree
   :members:
   :undoc-members:

.. doxygenclass:: kdtree::TetgenAdapter
   :project: KDTree
   :members:
   :undoc-members:

Utility Namespaces
------------------

.. doxygennamespace:: kdtree::util
   :project: KDTree
   :members:
   :undoc-members:

.. doxygennamespace:: kdtree::TreeNodeFactory
   :project: KDTree
   :members:
   :undoc-members:

Header Files
============

Core Headers
------------

.. doxygenfile:: KDTree.h
   :project: KDTree

.. doxygenfile:: TreeNode.h
   :project: KDTree

.. doxygenfile:: SplitNode.h
   :project: KDTree

.. doxygenfile:: LeafNode.h
   :project: KDTree

.. doxygenfile:: TreeNodeFactory.h
   :project: KDTree

Plane Selection
---------------

.. doxygenfile:: PlaneSelectionAlgorithm.h
   :project: KDTree

.. doxygenfile:: PlaneSelectionAlgorithmFactory.h
   :project: KDTree

.. doxygenfile:: LogNPlane.h
   :project: KDTree

.. doxygenfile:: LogNSquaredPlane.h
   :project: KDTree

.. doxygenfile:: SquaredPlane.h
   :project: KDTree

.. doxygenfile:: NoTreePlane.h
   :project: KDTree

.. doxygenfile:: PlaneEventAlgorithm.h
   :project: KDTree

Geometry
--------

.. doxygenfile:: GeometryObject.h
   :project: KDTree

.. doxygenfile:: Box.h
   :project: KDTree

.. doxygenfile:: Plane.h
   :project: KDTree

.. doxygenfile:: PlaneEvent.h
   :project: KDTree

.. doxygenfile:: SplitParam.h
   :project: KDTree

Utilities
---------

.. doxygenfile:: TetgenAdapter.h
   :project: KDTree

.. doxygenfile:: UtilityContainer.h
   :project: KDTree

.. doxygenfile:: UtilityFloatArithmetic.h
   :project: KDTree

.. doxygenfile:: UtilityThrust.h
   :project: KDTree

.. doxygenfile:: Info.h
   :project: KDTree

.. doxygenfile:: KdDefinitions.h
   :project: KDTree

Indices and tables
==================

* :ref:`genindex`
* :ref:`search`
