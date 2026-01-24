.. KDTree documentation master file, created by
   sphinx-quickstart on Sun Nov 23 23:01:20 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. KDTree documentation master file

Welcome to KDTree's documentation!
==================================

KDTree is a spatial data structure implementation for efficient nearest neighbor searches and spatial queries. It provides efficient spatial indexing and querying capabilities for 3D geometric objects. The implementation is written in C++ and designed for high performance applications. Python bindings are included for ease of use.

Here is a visualization of a KDTree partitioning a 3D space filled with particles:

.. figure:: _static/kdtree_3d.png
   :alt: KDTree Visualization
   :align: center
   :width: 400px
|

This project supports partiioning of particles in 3D space as well as triangle meshes (polyhedra).

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
