.. KDTree documentation master file, created by
   sphinx-quickstart on Sun Nov 23 23:01:20 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. KDTree documentation master file

KDTree
=======

A high-performance, lazily-built KD-Tree implementation for spatial partitioning of particles and triangle meshes in three-dimensional space. This library provides efficient ray-triangle intersection queries and is particularly suited for polyhedral geometry operations.

Here is a visualization of a KDTree partitioning a 3D space filled with particles:

.. figure:: _static/kdtree_3d.png
   :alt: KDTree Visualization
   :align: center
   :width: 400px

This project supports partioning of particles in 3D space as well as triangle meshes (polyhedra).

.. toctree::
   :maxdepth: 2
   :caption: KDTree

   overview

.. toctree::
   :maxdepth: 1
   :caption: Minimal Code Examples:
   :glob:

   examples/*

.. toctree::
   :maxdepth: 1
   :caption: Source Code Documentation:
   :glob:

   code_docs/*

.. toctree::
   :maxdepth: 1
   :caption: Build and Configuration:

   cmake/cmake

.. toctree::
   :maxdepth: 1
   :caption: Building and Running the Tests:

   tests/ctest
   tests/pytest
