C++
----

Installation and Usage
^^^^^^^^^^^^^^^^^^^^^^^

Installation is straightforward using CMake. After cloning the repository, create a build directory and run the following commands:

.. code-block:: console

   mkdir build
   cd build
   cmake ..
   make

Here is a simple example demonstrating how to create a KDTree using particles and triangle meshes:

.. code-block::

   #include "KDTree/tree/KDTree.h"

   #include <array>
   #include <iostream>
   #include <vector>

   int main() {
      std::vector<std::array<double, 3>> particles{
         {2.0, 3.0, 6.0}, {5.0, 4.0, 7.0}, {9.0, 6.0, 1.0},
         {4.0, 7.0, 2.0}, {8.0, 1.0, 5.0}, {7.0, 2.0, 4.0}
      };

      std::vector<std::array<double, 3>> vertices{
          {2.0, 3.0, 6.0}, {5.0, 4.0, 7.0}, {9.0, 6.0, 1.0},
          {4.0, 7.0, 2.0}, {8.0, 1.0, 5.0}, {7.0, 2.0, 4.0}
      };

      std::vector<std::array<int, 3>> faces{
          {0, 1, 2}, {3, 4, 5}
      };

      // building KDTree with particles
      kdtree::KDTree tree_particles{particles};

      // building KDTree with mesh
      kdtree::KDTree tree_mesh{vertices, faces};

      // building KDTree using mesh file paths
      kdtree::KDTree tree_from_files{"path/to/mesh.node", "path/to/mesh.face"};

      std::cout << "KDTree: " << tree_particles << std::endl;
      std::cout << "KDTree: " << tree_mesh << std::endl;
      std::cout << "KDTree: " << tree_from_files << std::endl;

      return 0;
   }

For advanced usage and configuration options, please refer to the detailed documentation in :ref:`KDTree`.