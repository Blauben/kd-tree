Python Documentation
====================

.. py:module:: scikdtree

   Python bindings for the KDTree C++ library, generated via `nanobind <https://nanobind.readthedocs.io>`_.
   The module is also available at the package level via ``KDTree_Python``.

   .. code-block:: python

      from scikdtree import KDTree, PlaneSelectionAlgorithm

PlaneSelectionAlgorithm
-----------------------

.. py:class:: PlaneSelectionAlgorithm

   Enum controlling which split-plane algorithm the KDTree uses during construction.

   .. py:attribute:: LOG
      :type: PlaneSelectionAlgorithm

      O(n log n) — default, recommended for most use cases.

   .. py:attribute:: LOGSQUARED
      :type: PlaneSelectionAlgorithm

      O(n log² n) — unoptimised LOG variant, useful for validation and comparison.

   .. py:attribute:: QUADRATIC
      :type: PlaneSelectionAlgorithm

      O(n²) — suitable only for very small datasets.

   .. py:attribute:: NOTREE
      :type: PlaneSelectionAlgorithm

      No tree structure is built; falls back to brute-force traversal.
      Useful as a performance baseline.

KDTree
------

.. py:class:: KDTree(vertices, faces, algorithm=PlaneSelectionAlgorithm.LOG)
              KDTree(polySource, algorithm=PlaneSelectionAlgorithm.LOG)
              KDTree(nodeFilePath, faceFilePath, algorithm=PlaneSelectionAlgorithm.LOG)

   A lazily-built KD-Tree for spatial partitioning of triangle meshes or particle clouds in
   three-dimensional space. Thread-safe for concurrent intersection queries.

   The tree is built on demand: the root node is only constructed when the first query is made,
   or explicitly via :py:meth:`prebuildTree`.

   :param vertices: Vertex coordinates of the polyhedron. Each vertex is a list of three floats
      ``[x, y, z]``.
   :type vertices: list[list[float]]
   :param faces: Triangle faces of the polyhedron. Each face is a list of three vertex indices
      ``[i, j, k]``.
   :type faces: list[list[int]]
   :param polySource: A tuple ``(vertices, faces)`` combining both inputs above.
   :type polySource: tuple[list[list[float]], list[list[int]]]
   :param nodeFilePath: Path to a Tetgen ``.node`` file containing vertex coordinates.
   :type nodeFilePath: str
   :param faceFilePath: Path to a Tetgen ``.face`` file containing triangle indices.
   :type faceFilePath: str
   :param algorithm: Split-plane selection algorithm. Defaults to
      :py:attr:`PlaneSelectionAlgorithm.LOG`.
   :type algorithm: PlaneSelectionAlgorithm

   .. py:method:: prebuildTree() -> KDTree

      Eagerly constructs the entire KD-Tree instead of building it lazily on the first query.
      Useful when consistent query latency is required or when the same tree is queried many times.

      :returns: The same ``KDTree`` instance (allows method chaining).
      :rtype: KDTree

   .. py:method:: countIntersections(origin, ray) -> int

      Counts how many triangles the given ray intersects.

      :param origin: Ray origin as ``[x, y, z]``.
      :type origin: list[float]
      :param ray: Ray direction as ``[x, y, z]``.
      :type ray: list[float]
      :returns: Number of intersected triangles.
      :rtype: int

   .. py:method:: getIntersections(origin, ray) -> list[list[float]]

      Returns the exact intersection points of the given ray with the mesh triangles.

      :param origin: Ray origin as ``[x, y, z]``.
      :type origin: list[float]
      :param ray: Ray direction as ``[x, y, z]``.
      :type ray: list[float]
      :returns: List of intersection points, each as ``[x, y, z]``.
      :rtype: list[list[float]]

   .. py:method:: printTree() -> None

      Prints the KD-Tree structure to the Python console.

   .. py:method:: __str__() -> str

      Returns a string representation of the KD-Tree structure.

      :rtype: str
