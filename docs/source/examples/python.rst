Python
------

Installation and Usage
^^^^^^^^^^^^^^^^^^^^^^^

Installation is straightforward using pip. After cloning the repository, run the following command:

.. code-block:: console

   pip install .

Here is a simple example demonstrating how to create a KDTree using particles and triangle meshes:

.. code-block:: python

   from KDTree_Python import KDTree, PlaneSelectionAlgorithm

   # Define particles
   particles = [
       [2.0, 3.0, 6.0], [5.0, 4.0, 7.0], [9.0, 6.0, 1.0],
       [4.0, 7.0, 2.0], [8.0, 1.0, 5.0], [7.0, 2.0, 4.0]
   ]

   # Define vertices and faces
   vertices = [
       [2.0, 3.0, 6.0], [5.0, 4.0, 7.0], [9.0, 6.0, 1.0],
       [4.0, 7.0, 2.0], [8.0, 1.0, 5.0], [7.0, 2.0, 4.0]
   ]

   faces = [
       [0, 1, 2], [3, 4, 5]
   ]

   # Building KDTree with mesh
   tree_mesh = KDTree(vertices, faces)

   # Building KDTree with mesh and specific algorithm
   tree_mesh_log = KDTree(vertices, faces, PlaneSelectionAlgorithm.LOG)

   # Building KDTree using mesh file paths
   tree_from_files = KDTree("path/to/mesh.node", "path/to/mesh.face")

   # Prebuild the entire tree (optional)
   tree_mesh.prebuildTree()

   # Perform ray intersection query
   origin = [0.0, 0.0, 0.0]
   ray = [1.0, 0.0, 0.0]

   # Count intersections
   count = tree_mesh.countIntersections(origin, ray)
   print(f"Number of intersections: {count}")

   # Get intersection points
   intersections = tree_mesh.getIntersections(origin, ray)
   print(f"Intersection points: {intersections}")

   # Print tree structure
   print(f"KDTree: {tree_mesh}")

For advanced usage and configuration options, please refer to the detailed documentation in :ref:`KDTree`.
