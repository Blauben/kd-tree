Python
------

Installation and Usage
^^^^^^^^^^^^^^^^^^^^^^^

Installation is straightforward using pip. After cloning the repository, run the following command:

.. code-block:: console

   pip install .

Here is a simple example demonstrating how to create a KDTree using particles and triangle meshes:
- First import the necessary classes from the KDTree_Python module.
.. code-block:: python

   from KDTree_Python import KDTree, PlaneSelectionAlgorithm

Then define the data for particles as follows:
.. code-block:: python
   # Define particles
   particles = [
       [2.0, 3.0, 6.0], [5.0, 4.0, 7.0], [9.0, 6.0, 1.0],
       [4.0, 7.0, 2.0], [8.0, 1.0, 5.0], [7.0, 2.0, 4.0]
   ]

When working with triangle meshes, define the vertices and faces in a similar manner:
.. code-block:: python
   # Define vertices and faces
   vertices = [
       [2.0, 3.0, 6.0], [5.0, 4.0, 7.0], [9.0, 6.0, 1.0],
       [4.0, 7.0, 2.0], [8.0, 1.0, 5.0], [7.0, 2.0, 4.0]
   ]

   faces = [
       [0, 1, 2], [3, 4, 5]
   ]

Next, build the KDTree using the defined data:
- For particles:
.. code-block:: python
    # Building KDTree with particles
    tree_particles = KDTree(particles)

- For triangle meshes:
.. code-block:: python
   # Building KDTree with mesh
   tree_mesh = KDTree(vertices, faces)

- It is also possible to build a KDTree directly from .node and .face files containing the mesh data. This can be done as follows:
.. code-block:: python
   # Building KDTree using mesh file paths
   tree_from_files = KDTree("path/to/mesh.node", "path/to/mesh.face")

The KDTree can be used for various queries, such as counting intersections with a ray or retrieving intersection points. Here is an example of how to perform these operations:
.. code-block:: python
   # Prebuild the entire tree (optional), default is lazy building
   tree.prebuildTree()

   # Perform ray intersection query
   origin = [0.0, 0.0, 0.0]
   ray = [1.0, 0.0, 0.0]

   # Count intersections
   count = tree.countIntersections(origin, ray)
   print(f"Number of intersections: {count}")

   # Get intersection points
   intersections = tree.getIntersections(origin, ray)
   print(f"Intersection points: {intersections}")

   # Print tree structure
   print(f"KDTree: {tree}")

Sometimes it can be beneficial to specify a certain plane selection algorithm when building the tree. This can be done by passing an instance of the desired algorithm to the KDTree constructor. For example, to use the LogNPlane algorithm in the mesh example, replace the KDTree build section with the following code snippet:
.. code-block:: python
   # Building KDTree with a specific plane selection algorithm
   tree_mesh = KDTree(vertices, faces, PlaneSelectionAlgorithm.LOG)

For advanced usage and configuration options, please refer to the detailed documentation in :ref:`KDTree`.
