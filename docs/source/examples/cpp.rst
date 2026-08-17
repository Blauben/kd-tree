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

To customize the cmake build refer to the :ref:`cmake` documentation for detailed instructions on available build options and presets and typical commands.

Here are simple examples demonstrating how to create a KDTree using data in different formats:

In order to build a KDTree operating on particles refer to the following code snippet:

.. code-block::

   #include "KDTree/tree/KDTree.h"

   #include <array>
   #include <iostream>
   #include <vector>

   int main() {

      // data defintion section
      std::vector<std::array<double, 3>> particles{
         {2.0, 3.0, 6.0}, {5.0, 4.0, 7.0}, {9.0, 6.0, 1.0},
         {4.0, 7.0, 2.0}, {8.0, 1.0, 5.0}, {7.0, 2.0, 4.0}
      };

      // kdtree build section
      kdtree::KDTree tree_particles{particles};

      // optional usage example: prebuild the entire tree
      std::cout << "KDTree: " << tree_particles << std::endl;

      return 0;
   }

To build a KDTree operating on triangle meshes, replace the data definition and KDTree build sections with the following code snippet:

.. code-block::

   // data defintion section
   std::vector<std::array<double, 3>> vertices{
         {2.0, 3.0, 6.0}, {5.0, 4.0, 7.0}, {9.0, 6.0, 1.0},
         {4.0, 7.0, 2.0}, {8.0, 1.0, 5.0}, {7.0, 2.0, 4.0}
   };

   std::vector<std::array<int, 3>> faces{
         {0, 1, 2}, {3, 4, 5}
   };

   // kdtree build section
   kdtree::KDTree tree_mesh{vertices, faces};

It is also possible to store the mesh data in .node and .face files and pass the file paths to the KDTree constructor, skipping the data definition section entirely. Refer to the following code snippet:

.. code-block::

   // building KDTree using mesh file paths
   kdtree::KDTree tree_from_files{"path/to/mesh.node", "path/to/mesh.face"};

   // optional usage example: prebuild the entire tree
   std::cout << "KDTree: " << tree_from_files << std::endl;

For advanced usage and configuration options, please refer to the detailed documentation in :ref:`KDTree_cpp`.