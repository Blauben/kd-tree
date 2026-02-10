# KD-Tree

A high-performance, lazily-built KD-Tree implementation for spatial partitioning of particles and triangle meshes in three-dimensional space. This library provides efficient ray-triangle intersection queries and is particularly suited for polyhedral geometry operations.

## Features

- **Lazy Construction**: KD-Tree nodes are built on-demand, optimizing memory usage and initialization time
- **Multiple Split Algorithms**: Choose from several plane selection strategies:
  - `LOG` - O(n log n) complexity (default, recommended)
  - `LOGSQUARED` - O(n log² n) complexity
  - `QUADRATIC` - O(n²) complexity
  - `NOTREE` - No tree structure (brute force)
- **Parallel Execution**: Support for multiple parallelization backends:
  - Serial C++ (CPP)
  - OpenMP (OMP)
  - Intel Threading Building Blocks (TBB)
- **Thread-Safe**: Safe for concurrent ray intersection queries
- **Python Bindings**: Full Python interface via nanobind
- **Flexible Input**: Support for vertices/faces arrays, particle clouds, or Tetgen .node/.face files
- **Configurable Logging**: Multiple logging levels from TRACE to OFF

## Requirements

### C++ Library
- CMake >= 3.16
- C++20 compatible compiler
- Dependencies (automatically fetched):
  - spdlog (logging)
  - Thrust (parallel algorithms)
  - xsimd (SIMD operations)
  - TBB/OpenMP (optional, for parallelization)
  - Google Test (for testing)

### Python Interface
- Python >= 3.9
- scikit-build-core >= 0.4.3
- nanobind >= 1.3.2

## Installation

### Building the C++ Library

```bash
# Clone the repository
git clone <repository-url>
cd kd-tree

# Create build directory
mkdir build && cd build

# Configure with CMake
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DKD_TREE_PARALLELIZATION=TBB \
         -DKD_TREE_LOGGING_LEVEL=INFO

# Build
cmake --build . --config Release

# Run tests (optional)
ctest --output-on-failure
```

### Installing the Python Package

```bash
# Using pip
pip install .

# Or with conda environment
conda env create -f environment.yml
conda activate polyhedral-gravity-env
pip install .
```

## Usage

### C++ Example

```cpp
#include "KDTree/tree/KDTree.h"
#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"

using namespace kdtree;

// Option 1: From vertices and faces
std::vector<Vertex> vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
std::vector<IndexVector> faces = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
KDTree tree(vertices, faces, PlaneSelectionAlgorithm::Algorithm::LOG);

// Option 2: From .node and .face files
KDTree tree("path/to/model.node", "path/to/model.face", 
            PlaneSelectionAlgorithm::Algorithm::LOG);

// Option 3: From particles
std::vector<Vertex> particles = {{0, 0, 0}, {1, 1, 1}, {2, 2, 2}};
KDTree tree(particles, PlaneSelectionAlgorithm::Algorithm::LOG);

// Prebuild the entire tree (optional, bypasses lazy loading)
tree.prebuildTree();

// Perform ray intersection queries
Vertex origin = {0, 0, 0};
Vertex ray = {1, 0, 0};

// Count intersections
size_t count = tree.countIntersections(origin, ray);

// Get intersection points
std::set<Vertex> intersections;
tree.getIntersections(origin, ray, intersections);
```

### Python Example

```python
from KDTree_Python import KDTree, PlaneSelectionAlgorithm
import numpy as np

# Create vertices and faces
vertices = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]]
faces = [[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]]

# Build KD-Tree
tree = KDTree(vertices, faces, PlaneSelectionAlgorithm.LOG)

# Prebuild tree (optional)
tree.prebuildTree()

# Ray intersection query
origin = [0, 0, 0]
ray = [1, 0, 0]

# Count intersections
count = tree.countIntersections(origin, ray)
print(f"Number of intersections: {count}")

# Get intersection points
intersections = tree.getIntersections(origin, ray)
print(f"Intersection points: {intersections}")

# Print tree structure
tree.printTree()
```

## CMake Options

| Option | Default | Description |
|--------|---------|-------------|
| `BUILD_KD_TREE_EXECUTABLE` | ON | Build the C++ executable |
| `BUILD_KD_TREE_LIBRARY` | ON | Build the C++ library |
| `BUILD_KD_TREE_PYTHON_INTERFACE` | ON | Build Python bindings |
| `BUILD_KD_TREE_TESTS` | ON | Build unit tests |
| `BUILD_KD_TREE_DOCS` | OFF | Build documentation |
| `BUILD_KD_TREE_TIME_MEASUREMENT` | OFF | Build benchmark executable |
| `KD_TREE_PARALLELIZATION` | CPP | Parallelization backend (CPP/OMP/TBB) |
| `KD_TREE_LOGGING_LEVEL` | INFO | Logging level (TRACE/DEBUG/INFO/WARN/ERROR/CRITICAL/OFF) |

## Algorithm Comparison

| Algorithm | Time Complexity | Best Use Case                                  |
|-----------|----------------|------------------------------------------------|
| LOG | O(n log n) | Recommended for most cases                     |
| LOGSQUARED | O(n log² n) | Unoptimized LOG variant, useful for validation |
| QUADRATIC | O(n²) | Small datasets only                            |
| NOTREE | O(n) build, O(n) query | Baseline comparison                            |

## Project Structure

```
kd-tree/
├── src/
│   ├── KDTree/              # Core C++ library
│   │   ├── tree/            # KD-Tree implementation
│   │   ├── model/           # Geometry models
│   │   ├── plane_selection/ # Split plane algorithms
│   │   ├── input/           # File I/O (Tetgen support)
│   │   └── util/            # Utilities
│   ├── KDTreePython/        # Python bindings
│   ├── kd_tree_main.cpp     # C++ example application
│   └── kd_time_main.cpp     # Benchmark application
├── test/                    # Unit tests
├── docs/                    # Documentation
├── resources/               # Example mesh files
└── cmake/                   # CMake modules
```

## Performance Tips

1. **Use the LOG algorithm**: Best balance of build time and query performance
2. **Prebuild for multiple queries**: Call `prebuildTree()` if you'll perform many intersection tests
3. **Enable parallelization**: Use TBB or OpenMP for large meshes
4. **Reduce logging in production**: Set `KD_TREE_LOGGING_LEVEL` to ERROR or OFF for maximum performance

## Testing

```bash
# Run all tests
cd build
ctest --output-on-failure

# Run specific test
./test/kdtree_tests
```

## Benchmarking

```bash
# Build with benchmarks enabled
cmake .. -DBUILD_KD_TREE_TIME_MEASUREMENT=ON
cmake --build .

# Run benchmarks
./KDTree_time
```

## License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.

## Author

Ben Frauenknecht (ben.Frauenknecht@tum.de)

## Acknowledgments

CMake configuration adapted from the [ESA Polyhedral Gravity Model](https://github.com/esa/polyhedral-gravity-model) project.

Original authors: Schuhmacher, J., Blazquez, E., Gratl, F., Izzo, D., & Gómez, P.

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## Citation

If you use this library in your research, please cite it appropriately.

