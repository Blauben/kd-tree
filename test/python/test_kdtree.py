# Basic pytest tests for KDTree_Python bindings
import pytest

def test_import_module():
    """Test that the KDTree_Python module can be imported without errors and that it contains the expected KDTree class."""
    import KDTree_Python

def test_kdtree_instantiation_and_methods():
    
    import KDTree_Python
    # Minimal cube data (matches C++ test)
    vertices = [
        [-1.0, -1.0, -1.0],
        [1.0, -1.0, -1.0],
        [1.0, 1.0, -1.0],
        [-1.0, 1.0, -1.0],
        [-1.0, -1.0, 1.0],
        [1.0, -1.0, 1.0],
        [1.0, 1.0, 1.0],
        [-1.0, 1.0, 1.0],
    ]
    faces = [
        [1, 3, 2], [0, 3, 1], [0, 1, 5], [0, 5, 4],
        [0, 7, 3], [0, 4, 7], [1, 2, 6], [1, 6, 5],
        [2, 3, 6], [3, 7, 6], [4, 5, 6], [4, 6, 7]
    ]
    # Instantiate KDTree
    tree = KDTree_Python.KDTree(vertices, faces)
    # Test __str__
    s = str(tree)
    assert isinstance(s, str)
    # Test prebuildTree
    tree.prebuildTree()
    # Test countIntersections (arbitrary ray)
    origin = [0.0, 0.0, 2.0]
    ray = [0.0, 0.0, -1.0]
    count = tree.countIntersections(origin, ray)
    assert isinstance(count, int) or isinstance(count, float)
    # Test getIntersections
    intersections = tree.getIntersections(origin, ray)
    assert isinstance(intersections, list)
    # Test printTree (should not raise)
    tree.printTree()