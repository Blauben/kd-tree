# Basic pytest tests for KDTree_Python bindings
import pytest

def test_import_module():
    """Test that the KDTree_Python module can be imported without errors and that it contains the expected KDTree class."""
    import scikdtree

def test_kdtree_instantiation_and_methods():
    """Test that a KDTree can be instantiated and that its methods can be called without errors."""

    import scikdtree
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
    tree = scikdtree.KDTree(vertices, faces)
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


def test_kdtree_plane_iter():
    import scikdtree as sci
    mesh_path = "resources/Eros_scaled-1000"
    tree = sci.KDTree(mesh_path + ".node", mesh_path + ".face")
    plane_count = 0
    for _ in tree.planes():
        plane_count += 1
    assert plane_count == 30, "Expected 30 planes in the KDTree. Iterator returned none."
